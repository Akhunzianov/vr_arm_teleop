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
const leftCtrlView  = new ControllerView(scene, { handed: 'left' });
const comms = new Comms('/ws');

comms.onBinary = (buf) => pcView.ingest(buf);
comms.onJson = (msg) => {
  switch (msg.type) {
    case 'workspace': workspaceView.setBounds(msg.min, msg.max); break;
    case 'anchor':    workspaceView.setOrigin(msg.vr_position_of_robot_origin); break;
    case 'phase':     stateMachine.setPhase(msg.phase); break;
    case 'prompt':    stateMachine.applyPrompt(msg); break;
    case 'safety':    stateMachine.applySafety(msg); break;
    case 'robot':     /* HUD echo, ignored for now */ break;
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

scene.setAnimationLoop((time, frame) => {
  if (!frame) return;
  const ref = scene.renderer.xr.getReferenceSpace();
  if (!ref) return;

  const snap = input.read(frame, ref);
  rightHandView.update(snap.hands.right);
  leftCtrlView.update(snap.ctrls.left);

  const right = snap.hands.right;
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
    let triggerChanged = false;
    if (!leftTriggerHeld && triggerVal >= TRIGGER_ENGAGE_HI) {
      leftTriggerHeld = true;
      triggerChanged = true;
    } else if (leftTriggerHeld && triggerVal <= TRIGGER_RELEASE_LO) {
      leftTriggerHeld = false;
      triggerChanged = true;
    }
    if (triggerChanged) {
      comms.sendJson({
        type: 'trigger', hand: 'left', name: 'trigger',
        value: leftTriggerHeld ? 1.0 : 0.0,
      });
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
