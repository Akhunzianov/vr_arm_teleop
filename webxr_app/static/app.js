// Entry point. Wires together the JS modules, mirroring the server-side
// modularity: each concern lives in a file under ./modules and the
// top-level loop here only orchestrates.
//
// Punchlist 1-3: scene + websocket + point cloud + workspace + overlay +
// finger calibration. Hand state streams at ~30 Hz; X edge presses
// drive the calibration FSM on the server.

import { Scene } from './modules/scene.js';
import { Comms } from './modules/comms.js';
import { PointCloudView } from './modules/pointcloud_view.js';
import { WorkspaceView } from './modules/workspace_view.js';
import { Overlay } from './modules/overlay.js';
import { StateMachine } from './modules/state_machine.js';
import { InputReader } from './modules/input_reader.js';
import { HandView } from './modules/hand_view.js';
import { ControllerView } from './modules/controller_view.js';
import { allCurls, thumbAbduction } from './modules/hand_math.js';

const scene = new Scene(document.body);
const pcView = new PointCloudView(scene);
const workspaceView = new WorkspaceView(scene);
const overlay = new Overlay(scene);
const stateMachine = new StateMachine(overlay);
const input = new InputReader();
const rightHandView = new HandView(scene);                  // right bare hand
// "Ghost" hand: stays where the user disengaged so the operator can find
// the same pose again before re-engaging. Distinct colour + translucency
// so it's never mistaken for the live hand.
const ghostHandView = new HandView(scene, {
  color: 0x33ccff,
  opacity: 0.55,
  debugJointDots: false,
});
ghostHandView.setVisible(false);
const leftCtrlView  = new ControllerView(scene, { handed: 'left' });
const comms = new Comms('/ws');

comms.onBinary = (buf) => pcView.ingest(buf);
comms.onJson = (msg) => {
  switch (msg.type) {
    case 'workspace': workspaceView.setBounds(msg.min, msg.max); break;
    case 'anchor':    workspaceView.setOrigin(msg.vr_position_of_robot_origin); break;
    case 'phase':
      stateMachine.setPhase(msg.phase);
      // First entry into 'ready' (calibration just finished, never engaged
      // yet): start the live-tracking ghost so the operator has a target
      // pose to align to before pulling the trigger.
      if (msg.phase === 'ready' && !ghostSnapshot && !leftTriggerHeld) {
        enterStartingGhostMode();
      }
      break;
    case 'robot':
      // Robot HUD echo. Used to drive the starting-ghost's orientation
      // and finger pose so the user can align to the actual robot state.
      if (msg.wrist_orientation) {
        robotWristQHelmet = robotToHelmetQuat(msg.wrist_orientation);
      }
      if (msg.finger_curls) {
        robotCurls = [...msg.finger_curls];
      }
      break;
    case 'prompt':    stateMachine.applyPrompt(msg); break;
    case 'safety':    stateMachine.applySafety(msg); break;
    default:          console.log('[control]', msg);
  }
};

const HAND_SEND_PERIOD_MS = 1000 / 30;
let lastHandSendMs = 0;
const prevButtons = { x: false, y: false };

// Edge-detect the left trigger with hysteresis. Two thresholds avoid
// chatter when the user holds the trigger near the midpoint; the server
// makes the final engage decision but we save bandwidth by only sending
// the message when the perceived state changes.
const TRIGGER_ENGAGE_HI = 0.6;
const TRIGGER_RELEASE_LO = 0.3;
let leftTriggerHeld = false;

// Re-engage gate: after the first disengage we stash the hand pose and
// require the operator to return roughly to it before the next engage.
// Snapshot is a deep-cloned right-hand sample (with .wrist, .wristOrientation,
// .points) so it stays put while live samples keep arriving.
let ghostSnapshot = null;
const GHOST_POS_TOL_M = 0.08;        // ~8 cm
const GHOST_ORI_TOL_RAD = 25 * Math.PI / 180;

function cloneHandSnapshot(s) {
  if (!s || !s.valid) return null;
  return {
    valid: true,
    wrist: s.wrist ? [...s.wrist] : null,
    wristOrientation: s.wristOrientation ? [...s.wristOrientation] : null,
    points: s.points ? s.points.map(p => (p ? [...p] : null)) : null,
    orientations: s.orientations
      ? s.orientations.map(q => (q ? [...q] : null)) : null,
  };
}

// Starting-ghost mode: shown after calibration before the first engage.
// In this mode the ghost's wrist POSITION tracks the live user wrist (so
// position match is automatic), while its orientation and finger curls
// come from the robot's current state. The user only has to twist their
// wrist (and curl their fingers) to align before pulling the trigger.
let isStartingGhost = false;
let robotWristQHelmet = [0.0, 0.0, 0.0, 1.0];      // robot wrist quat, helmet frame
let robotCurls = [0.0, 0.0, 0.0, 0.0, 0.0];
let wristOutOfWorkspace = false;

// Robot frame (x=right, y=forward, z=up) -> helmet frame (x=right, y=up,
// z=back). Mirror of teleop_core/tracking.py: q_h = q_R_inv * q_r * q_R
// with q_R = (sin45, 0, 0, cos45).
const Q_HEL_TO_ROB = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
const Q_ROB_TO_HEL = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2];
// URDF wrist convention (fingers +Z, palm -Y) -> WebXR wrist convention
// (fingers +Y, palm -Z). Applied on the right so the resulting quat lives
// in the same convention as the live WebXR wrist sample, letting HandView's
// existing offsetQuaternion work unchanged for the ghost.
const Q_URDF_TO_WEBXR_WRIST = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
function quatMul(a, b) {
  return [
    a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
    a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
    a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
    a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
  ];
}
function robotToHelmetQuat(qr) {
  const out = quatMul(Q_ROB_TO_HEL, quatMul(qr, Q_HEL_TO_ROB));
  const n = Math.hypot(out[0], out[1], out[2], out[3]) || 1;
  return [out[0]/n, out[1]/n, out[2]/n, out[3]/n];
}

function enterStartingGhostMode() {
  isStartingGhost = true;
  // Seed an empty snapshot; the per-frame loop fills wrist/orientation.
  ghostSnapshot = {
    valid: true,
    wrist: [0, 1.1, -0.4],   // fallback if user hand isn't tracked yet
    wristOrientation: [0, 0, 0, 1],
    points: null,
    orientations: null,
  };
  ghostHandView.update(ghostSnapshot);
  ghostHandView.driveCurls(robotCurls);
  ghostHandView.setVisible(true);
}

function updateStartingGhost(liveRight) {
  if (!isStartingGhost) return;
  if (liveRight && liveRight.valid && liveRight.wrist) {
    ghostSnapshot.wrist = [...liveRight.wrist];
  }
  ghostSnapshot.wristOrientation = quatMul(robotWristQHelmet, Q_URDF_TO_WEBXR_WRIST);
  ghostHandView.update(ghostSnapshot);
  ghostHandView.driveCurls(robotCurls);
}

function clearGhost() {
  isStartingGhost = false;
  ghostSnapshot = null;
  ghostHandView.setVisible(false);
}

function poseMatchesGhost(live) {
  if (!ghostSnapshot || !live || !live.valid || !live.wrist) return false;
  const a = live.wrist, b = ghostSnapshot.wrist;
  const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  const posErr = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (posErr > GHOST_POS_TOL_M) return false;
  if (!live.wristOrientation || !ghostSnapshot.wristOrientation) return true;
  const p = live.wristOrientation, q = ghostSnapshot.wristOrientation;
  const dot = Math.abs(p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3]);
  const oriErr = 2 * Math.acos(Math.min(1, dot));
  return oriErr <= GHOST_ORI_TOL_RAD;
}

scene.setAnimationLoop((time, frame) => {
  if (!frame) return;
  const ref = scene.renderer.xr.getReferenceSpace();
  if (!ref) return;

  const snap = input.read(frame, ref);
  rightHandView.update(snap.hands.right);
  leftCtrlView.update(snap.ctrls.left);

  const right = snap.hands.right;
  // Drive the starting-ghost from the live wrist + latest robot state.
  if (isStartingGhost) updateStartingGhost(right);

  // Out-of-workspace warning. Client-side check against the box drawn
  // by WorkspaceView; only meaningful once we have both bounds and the
  // anchor, and the user's wrist is being tracked.
  if (right && right.valid && right.wrist) {
    const inside = workspaceView.containsPoint(right.wrist);
    if (inside === false && !wristOutOfWorkspace) {
      wristOutOfWorkspace = true;
      overlay.setWarning('Hand outside workspace — robot will stop at the border.', 'warn');
      workspaceView.setHighlight(true);
    } else if (inside === true && wristOutOfWorkspace) {
      wristOutOfWorkspace = false;
      overlay.setWarning('', 'warn');
      workspaceView.setHighlight(false);
    }
  }
  if (right && (time - lastHandSendMs) >= HAND_SEND_PERIOD_MS) {
    lastHandSendMs = time;
    comms.sendJson({
      type: 'hand',
      curls: allCurls(right.points),
      abduction: thumbAbduction(right.points),
      wrist_position: right.wrist || [0, 0, 0],
      wrist_orientation: right.wristOrientation || [0, 0, 0, 1],
      valid: right.valid,
    });
  }

  const leftCtrl = snap.ctrls.left;
  if (leftCtrl) {
    const xNow = leftCtrl.buttons.primary;
    const yNow = leftCtrl.buttons.secondary;
    if (xNow && !prevButtons.x) {
      comms.sendJson({ type: 'button', hand: 'left', name: 'x_click', pressed: true });
    }
    if (yNow && !prevButtons.y) {
      comms.sendJson({ type: 'button', hand: 'left', name: 'y_click', pressed: true });
    }
    prevButtons.x = xNow;
    prevButtons.y = yNow;

    const triggerVal = leftCtrl.buttons.trigger || 0.0;
    if (!leftTriggerHeld && triggerVal >= TRIGGER_ENGAGE_HI) {
      // Engage attempt. If a ghost is set, gate on returning to that
      // pose; otherwise (first ever engage) let it through.
      if (ghostSnapshot && !poseMatchesGhost(right)) {
        overlay.setPrompt(
          'Return your hand to the highlighted pose to re-engage tracking.',
          'warn',
        );
      } else {
        leftTriggerHeld = true;
        comms.sendJson({
          type: 'trigger', hand: 'left', name: 'trigger', value: 1.0,
        });
        // Successful engage: drop the ghost (starting or otherwise).
        // A new ghost is captured on the next release.
        clearGhost();
      }
    } else if (leftTriggerHeld && triggerVal <= TRIGGER_RELEASE_LO) {
      leftTriggerHeld = false;
      comms.sendJson({
        type: 'trigger', hand: 'left', name: 'trigger', value: 0.0,
      });
      // Stash the current right-hand pose so the operator can find their
      // way back before re-engaging.
      const snap = cloneHandSnapshot(right);
      if (snap) {
        ghostSnapshot = snap;
        ghostHandView.update(snap);
        ghostHandView.setVisible(true);
      }
    }
  }
});

const enterBtn = document.getElementById('enter');
if (enterBtn) {
  if (!navigator.xr) {
    enterBtn.disabled = true;
    enterBtn.textContent = 'WebXR unavailable';
  } else {
    enterBtn.addEventListener('click', async () => {
      try { await scene.startSession(); }
      catch (e) {
        console.warn('XR session start failed:', e);
        enterBtn.textContent = 'XR start failed (see console)';
      }
    });
  }
}
