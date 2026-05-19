# vr_arm_teleop

VR teleoperation of a robot arm + hand using a Meta Quest, fused
multi-camera depth, and Cartesian wrist tracking. Operator wears the
headset, sees a point-cloud reconstruction of the robot's workspace,
pulls the **left controller trigger** to engage tracking, and moves
their own right wrist to drive the robot's wrist.

This README is the orientation document for anyone (human or AI agent)
contributing. Read it once end-to-end before touching code.

---

## What we're building

End-to-end flow the operator experiences:

1. **Boot** the backend on the PC. RealSense cameras + a robot driver
   (sim or real) come up.
2. **Connect** from the Quest browser to the backend over HTTPS+WS.
3. **Calibrate fingers**: head-locked prompt panel walks the operator
   through 6 poses; their finger curl range is recorded. (Same flow
   we already validated in the SteamVR/ALVR prototype.)
4. **Point cloud appears** in front of the user — the fused output of
   the depth cameras pointed at the robot's workspace. A wireframe
   box shows the workspace bounds.
5. **Engage tracking** by holding the left trigger. We snapshot
   `(user_wrist_now, robot_wrist_now)` as the anchor pair.
6. **Track**: every frame, `target_robot_wrist = robot_anchor +
   (user_wrist_now - user_anchor)`, clamped to the workspace box.
   Finger curls also stream through, normalised against the
   calibration.
7. **Operator can walk around** — the robot has more reach than a
   standing human, so the operator literally steps to extend reach.
8. **Safety overlays** when the operator drifts outside the workspace
   (robot freezes at the border, red prompt appears) or moves faster
   than the robot can follow (yellow "lagging" prompt).

This is a *Cartesian* teleop: we never copy joint angles. Only the
wrist Cartesian pose + finger curls cross the boundary from human to
robot. The arm's IK takes care of the rest.

---

## Architecture

Three layers, one direction of dependency:

```
teleop_core/         pure interfaces + types + orchestration
   ↑
teleop_backends/     concrete implementations (cameras, robot)
   ↑
webxr_app/           entry point that wires backends together
                     + JS frontend served from static/
```

**Hard rule:** `teleop_core` never imports from `teleop_backends` or
`webxr_app`. If you're tempted, you're putting backend-specific knowledge
in the wrong place — make the interface richer instead.

### Why the layering

- Anyone can write a new `PointCloudSource` (e.g. ZED stereo, a
  pre-recorded clip, fully synthetic) and drop it in without touching
  anything else.
- Same for `RobotDriver`: pybullet sim, a real arm, a no-op logger
  for CI.
- The orchestrator (`TeleopServer`) is hardware-agnostic, so its
  state machine + safety logic gets tested with a `MockPointCloudSource`
  + `NoopRobotDriver` and we trust it before introducing real hardware
  noise.

### Interfaces to know

- **`teleop_core/point_cloud.py`** — `PointCloudSource.grab()` →
  `PointCloudFrame` (XYZ + RGB, world frame).
- **`teleop_core/robot.py`** — `RobotDriver.{start, stop, send,
  get_state, home_pose}`.
- **`teleop_core/messages.py`** — every JSON message that crosses the
  WebSocket. Frontend `modules/comms.js` mirrors this.

Pure logic modules — extend / fix in place, don't subclass:

- `workspace.py` — axis-aligned box, `contains` + `clamp`.
- `calibration.py` — `FingerCalibrationFSM` and the captured record.
- `tracking.py` — `CartesianTracker` (anchor + delta math).
- `safety.py` — `SafetyMonitor` (lag detection, workspace exit).
- `server.py` — `TeleopServer` orchestrator (the four async loops).

---

## Current status

**Everything is a skeleton.** Every method either declares its signature
+ docstring and `raise NotImplementedError`, or is an empty dataclass.
The architecture is locked in; implementations are open.

What you *can* run today: `python -m webxr_app` will start, fail to
construct the chosen backend (because they're all `NotImplementedError`),
and exit cleanly. The import graph works; the dependency direction is
clean.

What we already have working *in the old project* (`../vr_tendon_arm_teleop`)
that needs porting:

- Finger curl + thumb abduction math (`hand_pose.py` → mirrored on
  client in `modules/hand_math.js`).
- Calibration FSM (steps + prompts).
- WebSocket control protocol shape.
- Three.js scene + WebXR session setup.

---

## Punchlist (ordered)

Pick a numbered item, read the relevant interface files, implement it.
Each entry includes the files you'll touch and a rough effort.

### 1. Mock end-to-end ⏱ 3h

The smallest loop that proves the architecture works. No camera, no
robot.

- `teleop_core/point_cloud.encode_frame` — binary packer.
- `teleop_backends/pointcloud/mock.py` — synthetic animated cloud.
- `teleop_backends/robot/noop.py` — logs commands.
- `teleop_core/server.py` — `run()`, `_handle_ws`, `_pointcloud_loop`,
  `_control_loop`.
- `teleop_core/messages.py` — `encode` / `decode`.
- `webxr_app/__main__.py` — `_make_pc_source('mock')`,
  `_make_robot_driver('noop')`.

Frontend (smallest cut):
- `modules/comms.js`, `modules/scene.js`, `modules/pointcloud_view.js`,
  `app.js` wiring.

Done when: open `https://<lan-ip>:8000`, tap Enter VR, see a wavy
animated cloud floating in front of you.

### 2. Frame alignment + workspace box ⏱ 1h

Without this, points and boxes have no spatial meaning to the operator.

- `teleop_core/workspace.Workspace.{contains, clamp, as_dict}`.
- Frontend `modules/workspace_view.js` — wireframe at given bounds.
- Decide for v1: **anchor mode** (place the box at a fixed offset in
  WebXR `local-floor` space — e.g. `(0, 1.0, -0.6)` for "in front of
  where the headset booted, table height"). This is *approach 1* in
  the architecture doc below; sufficient until proper calibration.

Done when: workspace box visible at a plausible spot in front of the user.

### 3. Finger calibration ⏱ 30 min

Straight port from `../vr_tendon_arm_teleop`.

- `teleop_core/calibration.DEFAULT_STEPS`, `apply_curl`,
  `apply_abduction`, `FingerCalibrationFSM.{on_start, on_confirm,
  current_prompt, is_complete}`.
- Server hooks in `_control_loop` to advance on `ButtonMsg` (X click).
- Frontend `modules/state_machine.js` + `modules/overlay.js`.
- Frontend `modules/hand_math.js` (already a stub; port body from old
  project).

Done when: user can walk through all 6 calibration steps from VR,
ends in `phase: 'ready'`.

### 4. Pybullet robot driver ⏱ 3h

The first thing that actually responds to commanded wrist poses.

- `teleop_backends/robot/pybullet_driver.PybulletRobotDriver.*`.
- Copy `urdf_rc5_right_hand/` into this project (or keep a symlink) so
  it's self-contained.
- Use `p.calculateInverseKinematics` for arm; reuse the tendon coupling
  from old `vr_teleop/robot.py` for fingers.
- Run in `p.DIRECT` mode; the user only sees the robot via the point
  cloud / future URDF render.

Done when: in a unit script, you can construct the driver, call
`await send(RobotCommand(target_wrist_pose=...))` repeatedly, and
`await get_state()` returns end-effector poses close to commands.

### 5. Trigger to engage + Cartesian tracker ⏱ 2h

The actual teleop math.

- `teleop_core/types.Pose.{translated, identity}`.
- `teleop_core/tracking.CartesianTracker.{engage, disengage, update}`.
- Frontend `modules/input_reader.js` — read controllers each frame,
  edge-detect trigger.
- Send `TriggerMsg` on edges; server's `_control_loop` calls
  `tracker.engage(...)` / `disengage()`.
- `TeleopServer._command_loop` — at command_hz, build `RobotCommand`
  from tracker output + latest calibrated curls, call `driver.send(...)`.

Done when: hold left trigger, move right hand around, see the sim
robot's end-effector mirror your motion with the workspace clamp.

### 6. Safety monitor + warning overlay ⏱ 1.5h

- `teleop_core/safety.SafetyMonitor.step` — lag detection + workspace
  exit.
- `TeleopServer._safety_loop` — call step, broadcast `SafetyMsg`.
- Frontend overlay color + workspace-view highlight.

Done when: stepping outside the box flashes the box red and shows a
red prompt; the robot pauses at the boundary.

### 7. Multi-camera RealSense source ⏱ 3h

Swap the mock for real cameras.

- `teleop_backends/pointcloud/realsense_multi.MultiRealSenseSource.*`.
- Extrinsics config file: JSON of `{serial: 4x4_matrix}`. For v1,
  hand-tune by aligning known features in the rendered cloud.
- Workspace crop *inside the source* — drop points outside the
  configured box before encoding.

Done when: with N RealSenses on the workspace, the fused cloud in VR
looks like the actual workspace.

### 8. Real arm driver — *deferred*

`AeroArmDriver` is a stub today because we don't have the arm yet.
When that hardware ships:

- Connect arm SDK in `start()`.
- Solve / send wrist target in `send()`.
- Read back actual pose in `get_state()`.
- The Aero hand fingers piggy-back as in the old project.

The `TeleopServer` does not change. Same interface in, different
hardware out.

---

## Coordinate frames — read carefully

This is the easiest thing to get wrong. There are three frames:

| frame | origin | when used |
|---|---|---|
| `world` | robot base (by convention) | point cloud, robot, workspace |
| `play_space` | where the headset booted (WebXR `local-floor`) | user wrist samples from `XRFrame.getPose` |
| `view` | head, moves with user | head-locked text overlays |

Since the operator drives the robot via *deltas from an anchor*, you
**do not need an explicit `play_space → world` transform** for the
tracking math to work. The anchor pair (user_wrist, robot_wrist)
captured at trigger-down implicitly defines it for that engagement.

But you *do* need the transform to render things from the robot world
in the user's view (point cloud, workspace box). For v1 we cheat:

> The point cloud and workspace box are rendered at a fixed offset in
> `local-floor` space (configured to look plausible "in front of"
> where the headset booted). The operator chooses to stand somewhere
> that makes that geometry feel right.

Phase 2: a one-time recenter step where the operator stands at a known
position relative to the robot and presses a button to capture the
play-space→world transform.

Do not over-engineer this before step 5 works.

---

## Wire protocol

### Control channel (JSON text frames)

Each message has a `type` discriminator matching one of the dataclasses
in `teleop_core/messages.py`. The frontend has a 1:1 mirror in
`modules/comms.js` + `modules/state_machine.js`.

Client → Server:
- `HandStateMsg` — streamed at ~30 Hz, includes wrist position +
  orientation in play_space, finger curls, raw abduction.
- `ButtonMsg` — edge events (X to advance calibration, Y to quit).
- `TriggerMsg` — analog value; edges trigger engage/disengage.

Server → Client:
- `PhaseMsg` — `'idle' | 'finger_cal' | 'ready' | 'tracking' | 'fault'`.
- `PromptMsg` — head-locked text panel content + severity.
- `WorkspaceMsg` — one-time announcement of the workspace box bounds.
- `RobotEchoMsg` — live robot pose for HUD.
- `SafetyMsg` — discrete safety event.

### Point cloud channel (binary frames, same WebSocket)

```
[uint32 N][uint32 reserved=0]
[N × int16 x_mm][N × int16 y_mm][N × int16 z_mm]   # world frame, mm
[N × uint8 r][N × uint8 g][N × uint8 b]
```

~9 bytes / point. With WebSocket `permessage-deflate` and a workspace
crop, a 3-camera fused cloud (~10k points) compresses to ~30 KB/frame.
At 15 Hz = ~450 KB/s — trivial over Wi-Fi or USB.

We deferred delta encoding intentionally; revisit only if measured
bandwidth becomes a problem.

---

## Contributing — agent-facing guidance

If you are an AI agent picking this up, the rules of engagement:

1. **Pick one numbered item from the Punchlist.** Don't try to do
   several at once. The interfaces let you commit progress without
   breaking the rest of the project.
2. **Implement against the interface, never against a concrete class.**
   If you're tempted to `import pybullet` from inside `teleop_core`,
   stop. Add a method to the relevant ABC instead.
3. **Honor the wire format.** If you add a new control message, define
   the dataclass in `teleop_core/messages.py` *and* extend the
   frontend's `comms.js` switch. Do both in the same change.
4. **Don't bypass the dependency direction.** No imports from
   `teleop_core` into `teleop_backends`. No imports from anywhere
   else into `webxr_app`.
5. **No mutable globals in `teleop_core`.** All state lives on
   instance attributes of `TeleopServer`. This keeps the orchestrator
   testable and lets us run multiple instances side-by-side.
6. **Async-first.** All I/O is awaitable. Anything that has to block
   (rs.pipeline.wait_for_frames, pybullet stepSimulation) goes inside
   `asyncio.to_thread(...)`.
7. **Match existing port code.** When you port logic from
   `../vr_tendon_arm_teleop`, keep the numerics identical so the two
   projects produce the same calibrations.
8. **Don't add a new top-level dependency without updating
   `requirements.txt` and noting which backend needs it.** Core has
   only `aiohttp` + `numpy`.
9. **Comment why, not what.** The `what` is read in the code; the
   `why` is what an agent six months from now needs.

---

## File map

```
teleop_core/
  __init__.py         re-exports
  types.py            Pose, Vec3
  point_cloud.py      PointCloudFrame, PointCloudSource, encode_frame
  robot.py            RobotState, RobotCommand, RobotDriver
  workspace.py        Workspace (axis-aligned box, contains/clamp)
  calibration.py      FingerCalibrationFSM, CalibrationRecord, steps
  tracking.py         CartesianTracker, TrackingResult, WristAnchor
  safety.py           SafetyMonitor, SafetyEvent, SafetyKind, Severity
  messages.py         WebSocket message dataclasses + JSON codec
  server.py           TeleopServer orchestrator (four async loops)

teleop_backends/
  pointcloud/
    mock.py              synthetic animated cloud
    realsense_multi.py   N RealSenses fused into world frame
    pybullet_render.py   pybullet depth-render as a fake sensor
  robot/
    noop.py              logs commands, never moves
    pybullet_driver.py   sim robot via pybullet IK
    aero_arm.py          real Aero hand + (TBD arm)

webxr_app/
  __main__.py         CLI + backend wiring (the ONLY file that imports
                      from both teleop_core AND teleop_backends)
  static/
    index.html
    style.css
    app.js                 top-level wiring (currently throws)
    modules/
      comms.js             WebSocket client (JSON + binary)
      scene.js             three.js + XR session
      input_reader.js      per-frame WebXR input snapshot
      hand_view.js         tracked-hand sphere visualization
      pointcloud_view.js   THREE.Points bound to the binary stream
      workspace_view.js    workspace box wireframe
      overlay.js           head-locked text panels (prompt + warning)
      state_machine.js     reflects server phase
      hand_math.js         finger curl + thumb abduction (pure)
```

---

## Running

```bash
pip install -r requirements.txt

# Smallest demo (after step 1 of the punchlist is done):
python -m webxr_app --pc-backend mock --robot-backend noop

# Real cameras + sim robot (after steps 1, 4, 7):
python -m webxr_app --pc-backend realsense \
  --cameras config/cameras.json \
  --robot-backend pybullet --urdf urdf/Robot_with_right_hand_cor.urdf
```

For Wi-Fi access from the Quest, generate a self-signed cert and pass
`--cert / --key`. For wired:

```bash
adb reverse tcp:8000 tcp:8000
# then open http://localhost:8000 on the Quest browser
```

---

## What this project deliberately does *not* do

- Joint-by-joint copying of arm pose. We are Cartesian-only.
- Trying to encode every pixel of the cloud. We crop + quantize, and
  defer delta encoding.
- Sim-as-source-of-truth for hardware integration. The pybullet robot
  driver is for development; production runs on real hardware.
- Pretend that frame alignment is solved. Read the Coordinate Frames
  section carefully before assuming anything about where points
  "live" relative to the user.
