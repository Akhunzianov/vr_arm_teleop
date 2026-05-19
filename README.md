# vr_arm_teleop

VR teleoperation of a robot arm + hand using a Meta Quest, fused
multi-camera depth, and Cartesian wrist tracking. The operator wears
the headset, sees a point-cloud reconstruction of the workspace, holds
the **left controller trigger** to engage, and moves their own right
wrist to drive the robot's wrist. Finger curls stream through after a
one-time calibration.

This is a *Cartesian* teleop — only the wrist pose + finger curls
cross the human→robot boundary. The arm's IK takes care of the rest.

---

## What works today

The end-to-end mock loop runs on a single laptop: launch the server
with a sim robot, connect from a Quest browser, calibrate fingers,
and drive a pybullet wrist around in real time.

Implemented:

- **`teleop_core/`** — types, workspace (axis-aligned box with
  `contains` / `clamp`), finger calibration FSM (6-step prompt flow),
  `CartesianTracker` (anchor + delta math), WebSocket message
  dataclasses + JSON codec, and `TeleopServer` orchestrating the
  four async loops (control, command, point cloud, safety stub).
- **Point cloud backend** — `MockPointCloudSource` (synthetic
  animated cloud).
- **Robot backends** — `NoopRobotDriver` (logs commands),
  `PybulletRobotDriver` (full 6-DoF arm via `calculateInverseKinematics`
  + tendon-coupled hand fingers, URDF in `urdf_rc5_right_hand/`),
  `FloatingWristDriver` (6-DoF floating wrist for development without
  IK).
- **WebXR frontend** (`webxr_app/static/`) — Three.js scene + WebXR
  session, per-frame input reader, tracked-hand visualization,
  controller models, point-cloud renderer bound to the binary stream,
  workspace wireframe, head-locked overlay panels, calibration state
  machine, finger-curl + thumb-abduction math.
- **CLI wiring** (`webxr_app/__main__.py`) — picks pc/robot backends,
  derives the workspace box from the robot's home pose (or reads it
  from `--workspace path.json`), starts the HTTPS+WS server.

Not implemented yet — see [ROADMAP.md](ROADMAP.md):

- `MultiRealSenseSource` (real cameras)
- `PybulletPointCloudSource` (depth render from sim as a fake sensor)
- `AeroArmDriver` (real arm — hardware not on hand)
- `SafetyMonitor.step` + a couple of state-transition hooks in
  `TeleopServer`

---

## Quick start

```bash
git clone https://github.com/Akhunzianov/vr_arm_teleop.git
cd vr_arm_teleop
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Generate a self-signed cert so the Quest browser will let you enter
WebXR (WebXR requires HTTPS on LAN, or `localhost`):

```bash
mkdir -p certs
openssl req -x509 -newkey rsa:2048 -keyout certs/key.pem \
  -out certs/cert.pem -days 365 -nodes -subj "/CN=$(hostname -I | awk '{print $1}')"
```

Run the smallest end-to-end demo — mock point cloud + sim robot:

```bash
python -m webxr_app \
  --pc-backend mock \
  --robot-backend pybullet \
  --cert certs/cert.pem --key certs/key.pem
```

On the Quest browser, open `https://<your-LAN-ip>:8000`, accept the
self-signed cert warning, tap **Enter VR**. You should see an animated
point cloud and a workspace wireframe in front of you.

### Wired alternative (no cert needed)

```bash
adb reverse tcp:8000 tcp:8000
python -m webxr_app --pc-backend mock --robot-backend pybullet
# Then in the Quest browser: http://localhost:8000
```

### Operating it in VR

1. **Finger calibration** — head-locked panel walks through 6 poses;
   press **X** on the left controller to advance each step.
2. **Engage tracking** — hold the **left trigger**. The server
   snapshots `(user_wrist_now, robot_wrist_now)` as the anchor pair.
3. **Track** — move your right hand; the robot wrist follows by
   `target = robot_anchor + (user_wrist_now - user_anchor)`, clamped
   to the workspace box. Finger curls stream through normalised
   against the calibration.
4. **Disengage** — release the trigger.
5. **Quit** — press **Y**.

### CLI flags

| flag | default | meaning |
|---|---|---|
| `--pc-backend` | `mock` | `mock` / `realsense` (stub) / `pybullet` (stub) |
| `--robot-backend` | `pybullet` | `noop` / `pybullet` / `floating` / `aero` (stub) |
| `--urdf` | shipped URDF | URDF path for pybullet/floating |
| `--pybullet-gui` | off | Show pybullet GUI window |
| `--home-joints` | derived | 6 comma-separated radians, e.g. `0,-2.0,1.8,-1.4,1.57,0` |
| `--cameras` | – | `cameras.json` for `--pc-backend realsense` |
| `--workspace` | derived from home | `workspace.json` with `{"min":[x,y,z],"max":[x,y,z]}` |
| `--port` | `8000` | HTTP/HTTPS port |
| `--cert` / `--key` | – | TLS cert + key (required for non-localhost Quest) |

---

## Architecture

Three layers, one direction of dependency:

```
teleop_core/         pure interfaces + types + orchestration
   ↑
teleop_backends/     concrete implementations (cameras, robot)
   ↑
webxr_app/           CLI that wires backends together
                     + JS frontend served from static/
```

**Hard rule:** `teleop_core` never imports from `teleop_backends` or
`webxr_app`. If you're tempted, make the interface richer instead.

### Key interfaces

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
- `safety.py` — `SafetyMonitor` (lag detection, workspace exit — stub).
- `server.py` — `TeleopServer` orchestrator (the four async loops).

### Coordinate frames

| frame | origin | when used |
|---|---|---|
| `world` | robot base | point cloud, robot, workspace |
| `play_space` | where the headset booted (WebXR `local-floor`) | user wrist samples |
| `view` | head, moves with user | head-locked text overlays |

Because we drive the robot via *deltas from an anchor*, no explicit
`play_space → world` transform is needed for the tracking math — the
anchor pair captured at trigger-down implicitly defines it.

For rendering the point cloud and workspace box in the user's view,
v1 cheats: they're placed at a fixed offset in `local-floor` space.
The operator chooses to stand somewhere that makes the geometry feel
right. A proper recenter step is on the roadmap.

---

## Wire protocol

### Control channel (JSON text frames)

Each message has a `type` discriminator matching one of the
dataclasses in `teleop_core/messages.py`. The frontend has a 1:1
mirror in `modules/comms.js` + `modules/state_machine.js`.

Client → Server:
- `HandStateMsg` — streamed at ~30 Hz; wrist position + orientation
  in play_space, finger curls, raw abduction.
- `ButtonMsg` — edge events (X to advance calibration, Y to quit).
- `TriggerMsg` — analog value; edges drive engage/disengage.

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

~9 bytes/point. With WebSocket `permessage-deflate` and a workspace
crop, a 3-camera fused cloud (~10k points) compresses to ~30 KB/frame.
At 15 Hz ≈ 450 KB/s — trivial over Wi-Fi or USB.

---

## File map

```
teleop_core/
  types.py            Pose, Vec3
  point_cloud.py      PointCloudFrame, PointCloudSource, encode_frame
  robot.py            RobotState, RobotCommand, RobotDriver
  workspace.py        Workspace (axis-aligned box)
  calibration.py      FingerCalibrationFSM, CalibrationRecord, steps
  tracking.py         CartesianTracker, TrackingResult, WristAnchor
  safety.py           SafetyMonitor (stub), SafetyEvent
  messages.py         WebSocket message dataclasses + JSON codec
  server.py           TeleopServer orchestrator (four async loops)

teleop_backends/
  pointcloud/
    mock.py              synthetic animated cloud
    realsense_multi.py   N RealSenses fused into world frame [stub]
    pybullet_render.py   pybullet depth-render as a fake sensor [stub]
  robot/
    noop.py                  logs commands, never moves
    pybullet_driver.py       sim 6-DoF arm + tendon hand via pybullet IK
    floating_wrist_driver.py 6-DoF floating wrist (no IK), for dev
    aero_arm.py              real Aero hand + (TBD arm) [stub]

webxr_app/
  __main__.py         CLI + backend wiring (the ONLY file that imports
                      from both teleop_core AND teleop_backends)
  static/
    index.html / style.css / app.js
    modules/
      comms.js             WebSocket client (JSON + binary)
      scene.js             three.js + XR session
      input_reader.js      per-frame WebXR input snapshot
      hand_view.js         tracked-hand visualization
      controller_view.js   controller models
      pointcloud_view.js   THREE.Points bound to the binary stream
      workspace_view.js    workspace box wireframe
      overlay.js           head-locked text panels (prompt + warning)
      state_machine.js     reflects server phase
      hand_math.js         finger curl + thumb abduction (pure)

urdf_rc5_right_hand/  RC5 arm + Aero right hand URDF + meshes
config/               example workspace.json
scripts/              standalone smoke tests
```

---

## What this project deliberately does *not* do

- Joint-by-joint copying of arm pose. Cartesian-only.
- Encode every pixel of the cloud. We crop + quantize, and defer
  delta encoding until measured bandwidth becomes a problem.
- Treat the pybullet driver as the production target. It's for
  development; production runs on the real arm via `AeroArmDriver`
  once that hardware ships.
