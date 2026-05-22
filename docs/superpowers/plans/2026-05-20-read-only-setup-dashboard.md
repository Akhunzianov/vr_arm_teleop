# Read-Only Setup Dashboard V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a read-only desktop dashboard on `--dashboard-port 8001` that shares the current teleop process and renders robot URDF state, fused point cloud, workspace bounds, and anchored Quest head/right-wrist poses.

**Architecture:** Add a backend-agnostic telemetry hub in `teleop_core` that samples robot and point-cloud state once and exposes cached snapshots to the teleop websocket and all dashboard websocket clients. Extend the existing `TeleopServer` to start a second aiohttp app/runner on the dashboard port while keeping the Quest app on the existing port. Add a no-build dashboard frontend under `webxr_app/dashboard_static/` that uses Three.js, OrbitControls, and URDFLoader through import maps.

**Tech Stack:** Python 3.10, aiohttp, numpy, pytest, vanilla ES modules, Three.js `0.160.0`, `urdf-loader` `0.12.6`.

---

## Pulled State Notes

- The local branch has been fast-forwarded to `origin/main`.
- Baseline command after pull: `rtk python -m pytest -q`.
- Baseline result after pull: `19 passed in 1.11s`.
- Current main already includes RC5/Aero hardware driver code, ghost-hand re-engage logic, and `RobotCommand.target_thumb_abduction`.
- `RobotState` still exposes unnamed `joint_angles`; this plan adds named joint telemetry without breaking existing constructors.

## File Structure

- Modify `teleop_core/robot.py`: add optional `joint_names` and a safe `named_joint_angles` helper.
- Create `teleop_core/telemetry.py`: telemetry dataclasses, serialization helpers, shared `TelemetryHub`.
- Modify `teleop_core/messages.py`: extend `HandStateMsg` with headset pose fields.
- Modify `teleop_core/server.py`: create and run the telemetry hub, feed it XR/anchor updates, serve a second dashboard aiohttp app, and send Quest point clouds from the hub cache.
- Modify `teleop_core/__init__.py`: export the new telemetry types.
- Modify `teleop_backends/robot/pybullet_driver.py`: return named URDF joint states for all non-fixed joints.
- Modify `teleop_backends/robot/floating_wrist_driver.py`: return named URDF joint states for the hand model.
- Modify `teleop_backends/robot/aero_arm.py`: return empty named-joint telemetry for arm joints until the real API exposes joint reads; keep wrist/finger echo behavior unchanged.
- Modify `webxr_app/__main__.py`: add `--dashboard-port`, resolve the effective URDF path once, and pass it into `ServerConfig`.
- Modify `webxr_app/static/app.js`: send headset pose along with existing right-hand state.
- Create `webxr_app/dashboard_static/index.html`, `style.css`, `dashboard.js`, and focused modules for robot, point cloud, workspace, XR markers, and status panels.
- Create `tests/test_dashboard_telemetry.py`: hub and serialization tests.
- Extend `tests/test_webxr_cli_pointcloud.py`: dashboard CLI/config tests.
- Extend or create `tests/test_messages.py`: headset pose decode defaults and populated values.

---

### Task 1: Add Named Joint State Contract

**Files:**
- Modify: `teleop_core/robot.py`
- Test: `tests/test_dashboard_telemetry.py`

- [ ] **Step 1: Write failing tests for named joint serialization**

Add this to `tests/test_dashboard_telemetry.py`:

```python
import time

import numpy as np

from teleop_core.robot import RobotState
from teleop_core.types import Pose


def test_robot_state_named_joint_angles_pairs_names_with_values():
    state = RobotState(
        wrist_pose=Pose.identity(frame="world"),
        joint_angles=np.array([1.25, -0.5], dtype=np.float32),
        finger_curls=np.zeros(5, dtype=np.float32),
        timestamp=time.monotonic(),
        joint_names=("joint0", "right_index_pip"),
    )

    assert state.named_joint_angles == {
        "joint0": 1.25,
        "right_index_pip": -0.5,
    }


def test_robot_state_named_joint_angles_is_empty_when_lengths_do_not_match():
    state = RobotState(
        wrist_pose=Pose.identity(frame="world"),
        joint_angles=np.array([1.25], dtype=np.float32),
        finger_curls=np.zeros(5, dtype=np.float32),
        timestamp=time.monotonic(),
        joint_names=("joint0", "joint1"),
    )

    assert state.named_joint_angles == {}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: FAIL because `RobotState.__init__` does not accept `joint_names` or because `named_joint_angles` is missing.

- [ ] **Step 3: Implement minimal contract**

In `teleop_core/robot.py`, update `RobotState`:

```python
@dataclass(frozen=True)
class RobotState:
    """Snapshot of the robot at a point in time. Used for safety + HUD."""

    wrist_pose: Pose
    joint_angles: np.ndarray
    finger_curls: np.ndarray
    timestamp: float
    joint_names: tuple[str, ...] = ()

    @property
    def named_joint_angles(self) -> dict[str, float]:
        """Return URDF-name keyed joint angles when the driver provides names."""
        if len(self.joint_names) != int(self.joint_angles.shape[0]):
            return {}
        return {
            name: float(angle)
            for name, angle in zip(self.joint_names, self.joint_angles)
        }
```

- [ ] **Step 4: Verify tests pass**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: PASS.

---

### Task 2: Implement Shared Telemetry Hub

**Files:**
- Create: `teleop_core/telemetry.py`
- Modify: `teleop_core/__init__.py`
- Test: `tests/test_dashboard_telemetry.py`

- [ ] **Step 1: Add failing tests for snapshots, XR alignment, and point-cloud fanout**

Append to `tests/test_dashboard_telemetry.py`:

```python
import asyncio

from teleop_core.point_cloud import PointCloudFrame
from teleop_core.telemetry import TelemetryHub
from teleop_core.workspace import Workspace


class FakeRobot:
    def __init__(self):
        self.state = RobotState(
            wrist_pose=Pose(
                position=np.array([0.1, 0.2, 0.3], dtype=np.float64),
                orientation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
                frame="world",
            ),
            joint_angles=np.array([0.4, -0.2], dtype=np.float32),
            finger_curls=np.array([0.0, 0.1, 0.2, 0.3, 0.4], dtype=np.float32),
            timestamp=123.0,
            joint_names=("joint0", "right_index_pip"),
        )

    async def get_state(self):
        return self.state


class CountingPointCloud:
    def __init__(self):
        self.grab_count = 0
        self.frame = PointCloudFrame(
            points=np.array([[0.0, 0.0, 0.5]], dtype=np.float32),
            colors=np.array([[255, 0, 0]], dtype=np.uint8),
            timestamp=234.0,
        )

    async def grab(self):
        self.grab_count += 1
        return self.frame


def _workspace():
    return Workspace(
        min_corner=np.array([-1.0, -2.0, 0.0], dtype=np.float32),
        max_corner=np.array([1.0, 2.0, 1.0], dtype=np.float32),
    )


def test_dashboard_snapshot_contains_model_workspace_robot_and_unaligned_xr():
    async def run():
        hub = TelemetryHub(
            point_cloud_source=CountingPointCloud(),
            robot_driver=FakeRobot(),
            workspace=_workspace(),
            urdf_url="/robot/robot.urdf",
            urdf_assets_url="/robot/assets/",
            pointcloud_hz=1000.0,
            robot_hz=1000.0,
            status_hz=1000.0,
        )
        await hub.sample_robot_once()

        snap = hub.snapshot()

        assert snap["type"] == "snapshot"
        assert snap["model"]["urdf_url"] == "/robot/robot.urdf"
        assert snap["model"]["urdf_assets_url"] == "/robot/assets/"
        assert snap["workspace"]["min"] == [-1.0, -2.0, 0.0]
        assert snap["workspace"]["max"] == [1.0, 2.0, 1.0]
        assert snap["robot"]["joints"] == {
            "joint0": 0.4000000059604645,
            "right_index_pip": -0.20000000298023224,
        }
        assert snap["xr"]["aligned"] is False
        assert snap["xr"]["head"] is None
        assert snap["xr"]["right_wrist"] is None

    asyncio.run(run())


def test_dashboard_xr_pose_stays_unaligned_until_anchor_exists():
    hub = TelemetryHub(
        point_cloud_source=CountingPointCloud(),
        robot_driver=FakeRobot(),
        workspace=_workspace(),
        urdf_url="/robot/robot.urdf",
        urdf_assets_url="/robot/assets/",
    )
    hub.update_xr_pose(
        head_position=(0.0, 1.6, 0.0),
        head_orientation=(0.0, 0.0, 0.0, 1.0),
        right_wrist_position=(0.2, 1.1, -0.3),
        right_wrist_orientation=(0.0, 0.0, 0.0, 1.0),
        valid=True,
        timestamp=10.0,
    )
    assert hub.snapshot()["xr"]["aligned"] is False

    hub.update_anchor((0.1, 1.0, -0.2), timestamp=11.0)
    snap = hub.snapshot()

    assert snap["xr"]["aligned"] is True
    assert snap["xr"]["head"]["position"] == [0.0, 1.6, 0.0]
    assert snap["xr"]["right_wrist"]["position"] == [0.2, 1.1, -0.3]


def test_multiple_dashboard_cloud_waiters_share_one_grab():
    async def run():
        pc = CountingPointCloud()
        hub = TelemetryHub(
            point_cloud_source=pc,
            robot_driver=FakeRobot(),
            workspace=_workspace(),
            urdf_url="/robot/robot.urdf",
            urdf_assets_url="/robot/assets/",
        )

        await hub.sample_pointcloud_once()
        first = await hub.wait_for_pointcloud(after_sequence=0, timeout=0.01)
        second = await hub.wait_for_pointcloud(after_sequence=0, timeout=0.01)

        assert first is not None
        assert second is not None
        assert first.sequence == second.sequence == 1
        assert first.payload == second.payload
        assert pc.grab_count == 1

    asyncio.run(run())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: FAIL because `teleop_core.telemetry.TelemetryHub` is missing.

- [ ] **Step 3: Implement `teleop_core/telemetry.py`**

Create:

```python
from __future__ import annotations

import asyncio
import contextlib
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from .point_cloud import PointCloudSource, encode_frame
from .robot import RobotDriver, RobotState
from .workspace import Workspace


@dataclass(frozen=True)
class EncodedPointCloud:
    sequence: int
    payload: bytes
    timestamp: float
    n_points: int


def _vec(values) -> list[float]:
    return [float(v) for v in values]


def _pose_dict(position, orientation, timestamp: float) -> dict[str, Any]:
    return {
        "position": _vec(position),
        "orientation": _vec(orientation),
        "timestamp": float(timestamp),
    }


class TelemetryHub:
    """Caches live setup telemetry and fans it out to read-only clients."""

    def __init__(
        self,
        *,
        point_cloud_source: PointCloudSource,
        robot_driver: RobotDriver,
        workspace: Workspace,
        urdf_url: str,
        urdf_assets_url: str,
        pointcloud_hz: float = 15.0,
        robot_hz: float = 30.0,
        status_hz: float = 1.0,
    ) -> None:
        self._pc = point_cloud_source
        self._robot = robot_driver
        self._workspace = workspace
        self._urdf_url = urdf_url
        self._urdf_assets_url = urdf_assets_url
        self._pointcloud_hz = float(pointcloud_hz)
        self._robot_hz = float(robot_hz)
        self._status_hz = float(status_hz)
        self._robot_state: RobotState | None = None
        self._robot_error: str | None = None
        self._pointcloud: EncodedPointCloud | None = None
        self._pointcloud_error: str | None = None
        self._pointcloud_sequence = 0
        self._cloud_condition = asyncio.Condition()
        self._xr_pose: dict[str, Any] | None = None
        self._anchor: dict[str, Any] | None = None
        self._tasks: list[asyncio.Task] = []
        self._stopping = False

    async def start(self) -> None:
        if self._tasks:
            return
        self._stopping = False
        self._tasks = [
            asyncio.create_task(self._robot_loop(), name="telemetry_robot_loop"),
            asyncio.create_task(self._pointcloud_loop(), name="telemetry_pointcloud_loop"),
        ]

    async def stop(self) -> None:
        self._stopping = True
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task
        self._tasks = []

    async def sample_robot_once(self) -> None:
        try:
            self._robot_state = await self._robot.get_state()
            self._robot_error = None
        except Exception as exc:
            self._robot_error = repr(exc)

    async def sample_pointcloud_once(self) -> None:
        try:
            frame = await self._pc.grab()
            self._pointcloud_error = None
        except Exception as exc:
            frame = None
            self._pointcloud_error = repr(exc)
        if frame is None:
            return
        payload = encode_frame(frame)
        async with self._cloud_condition:
            self._pointcloud_sequence += 1
            self._pointcloud = EncodedPointCloud(
                sequence=self._pointcloud_sequence,
                payload=payload,
                timestamp=float(frame.timestamp),
                n_points=int(frame.n_points),
            )
            self._cloud_condition.notify_all()

    async def wait_for_pointcloud(
        self,
        *,
        after_sequence: int,
        timeout: float,
    ) -> EncodedPointCloud | None:
        async with self._cloud_condition:
            if self._pointcloud is not None and self._pointcloud.sequence > after_sequence:
                return self._pointcloud
            try:
                await asyncio.wait_for(
                    self._cloud_condition.wait_for(
                        lambda: (
                            self._pointcloud is not None
                            and self._pointcloud.sequence > after_sequence
                        )
                    ),
                    timeout=timeout,
                )
            except asyncio.TimeoutError:
                return None
            return self._pointcloud

    def update_xr_pose(
        self,
        *,
        head_position,
        head_orientation,
        right_wrist_position,
        right_wrist_orientation,
        valid: bool,
        timestamp: float,
    ) -> None:
        if not valid:
            self._xr_pose = None
            return
        self._xr_pose = {
            "head": _pose_dict(head_position, head_orientation, timestamp),
            "right_wrist": _pose_dict(
                right_wrist_position,
                right_wrist_orientation,
                timestamp,
            ),
        }

    def update_anchor(self, vr_position_of_robot_origin, *, timestamp: float) -> None:
        self._anchor = {
            "vr_position_of_robot_origin": _vec(vr_position_of_robot_origin),
            "timestamp": float(timestamp),
        }

    def snapshot(self) -> dict[str, Any]:
        robot = self._robot_state
        cloud = self._pointcloud
        return {
            "type": "snapshot",
            "model": {
                "urdf_url": self._urdf_url,
                "urdf_assets_url": self._urdf_assets_url,
            },
            "workspace": {
                "min": _vec(self._workspace.min_corner),
                "max": _vec(self._workspace.max_corner),
                "frame": self._workspace.frame,
            },
            "robot": {
                "wrist": None if robot is None else _pose_dict(
                    robot.wrist_pose.position,
                    robot.wrist_pose.orientation,
                    robot.timestamp,
                ),
                "joints": {} if robot is None else robot.named_joint_angles,
                "finger_curls": [] if robot is None else _vec(robot.finger_curls),
                "timestamp": None if robot is None else float(robot.timestamp),
                "error": self._robot_error,
            },
            "pointcloud": {
                "sequence": 0 if cloud is None else int(cloud.sequence),
                "timestamp": None if cloud is None else float(cloud.timestamp),
                "n_points": 0 if cloud is None else int(cloud.n_points),
                "error": self._pointcloud_error,
            },
            "xr": {
                "aligned": self._xr_pose is not None and self._anchor is not None,
                "anchor": self._anchor,
                "head": None if self._xr_pose is None else self._xr_pose["head"],
                "right_wrist": None if self._xr_pose is None else self._xr_pose["right_wrist"],
            },
            "status": {
                "server_time": time.monotonic(),
                "robot_hz": self._robot_hz,
                "pointcloud_hz": self._pointcloud_hz,
                "status_hz": self._status_hz,
            },
        }

    async def _robot_loop(self) -> None:
        period = 1.0 / max(self._robot_hz, 1e-3)
        while not self._stopping:
            start = time.monotonic()
            await self.sample_robot_once()
            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    async def _pointcloud_loop(self) -> None:
        period = 1.0 / max(self._pointcloud_hz, 1e-3)
        while not self._stopping:
            start = time.monotonic()
            await self.sample_pointcloud_once()
            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))
```

- [ ] **Step 4: Export telemetry types**

In `teleop_core/__init__.py`, import and add to `__all__`:

```python
from .telemetry import EncodedPointCloud, TelemetryHub
```

```python
"EncodedPointCloud", "TelemetryHub",
```

- [ ] **Step 5: Verify telemetry tests pass**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: PASS.

---

### Task 3: Update Robot Drivers to Provide URDF Joint Names

**Files:**
- Modify: `teleop_backends/robot/pybullet_driver.py`
- Modify: `teleop_backends/robot/floating_wrist_driver.py`
- Modify: `teleop_backends/robot/aero_arm.py`
- Test: `tests/test_dashboard_telemetry.py`

- [ ] **Step 1: Add focused helper tests for driver-independent telemetry behavior**

Append:

```python
def test_dashboard_robot_snapshot_omits_joints_when_driver_has_no_names():
    async def run():
        robot = FakeRobot()
        robot.state = RobotState(
            wrist_pose=Pose.identity(frame="world"),
            joint_angles=np.zeros(6, dtype=np.float32),
            finger_curls=np.zeros(5, dtype=np.float32),
            timestamp=42.0,
        )
        hub = TelemetryHub(
            point_cloud_source=CountingPointCloud(),
            robot_driver=robot,
            workspace=_workspace(),
            urdf_url="/robot/robot.urdf",
            urdf_assets_url="/robot/assets/",
        )
        await hub.sample_robot_once()
        assert hub.snapshot()["robot"]["joints"] == {}

    asyncio.run(run())
```

- [ ] **Step 2: Run the helper tests**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: PASS after Task 1 and Task 2; this test locks the behavior that real drivers may omit names safely.

- [ ] **Step 3: Implement PyBullet named joints**

In `PybulletRobotDriver.__init__`, add:

```python
self._joint_name_by_index: dict[int, str] = {}
self._state_joint_indices: list[int] = []
self._state_joint_names: tuple[str, ...] = ()
```

In `start()` while iterating `getJointInfo`, fill all non-fixed joints:

```python
self._joint_name_by_index[i] = name
if jtype != p.JOINT_FIXED:
    self._movable_joint_indices.append(i)
    self._state_joint_indices.append(i)
```

After the loop:

```python
self._state_joint_names = tuple(
    self._joint_name_by_index[i] for i in self._state_joint_indices
)
```

In `get_state()`, read every `_state_joint_indices` entry:

```python
states = p.getJointStates(
    self._body_id,
    self._state_joint_indices,
    physicsClientId=self._client_id,
)
joint_q = np.array([s[0] for s in states], dtype=np.float32)
return RobotState(
    wrist_pose=Pose(...),
    joint_angles=joint_q,
    finger_curls=self._last_curls.copy(),
    timestamp=time.monotonic(),
    joint_names=self._state_joint_names,
)
```

- [ ] **Step 4: Implement FloatingWrist named joints**

Use the same `_joint_name_by_index`, `_state_joint_indices`, and `_state_joint_names` pattern in `FloatingWristDriver`. Return `joint_angles` from `p.getJointStates(...)` instead of `np.zeros(0, dtype=np.float32)`.

- [ ] **Step 5: Keep AeroArm safe and honest**

In `AeroArmDriver.get_state()`, keep the current `joint_angles=np.zeros(6, dtype=np.float32)` but add:

```python
joint_names=(),
```

This makes the dashboard show the wrist pose and finger echo while clearly not applying named FK for the real arm until a real joint-read API is connected.

- [ ] **Step 6: Run current tests**

Run: `rtk python -m pytest -q`

Expected: PASS.

---

### Task 4: Extend WebXR Hand Messages with Headset Pose

**Files:**
- Modify: `teleop_core/messages.py`
- Modify: `webxr_app/static/app.js`
- Test: `tests/test_messages.py`

- [ ] **Step 1: Add failing decode tests**

Create `tests/test_messages.py`:

```python
import json

from teleop_core.messages import HandStateMsg, decode


def test_hand_state_decode_has_head_pose_defaults():
    msg = decode(json.dumps({"type": "hand", "valid": True}))

    assert isinstance(msg, HandStateMsg)
    assert msg.head_position == (0.0, 0.0, 0.0)
    assert msg.head_orientation == (0.0, 0.0, 0.0, 1.0)
    assert msg.head_valid is False


def test_hand_state_decode_accepts_head_pose_fields():
    msg = decode(json.dumps({
        "type": "hand",
        "valid": True,
        "wrist_position": [0.1, 0.2, 0.3],
        "wrist_orientation": [0.0, 0.0, 0.0, 1.0],
        "head_position": [1.0, 1.5, -0.2],
        "head_orientation": [0.0, 0.707, 0.0, 0.707],
        "head_valid": True,
    }))

    assert msg.head_position == (1.0, 1.5, -0.2)
    assert msg.head_orientation == (0.0, 0.707, 0.0, 0.707)
    assert msg.head_valid is True
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `rtk python -m pytest tests/test_messages.py -q`

Expected: FAIL because `head_position`, `head_orientation`, and `head_valid` are missing.

- [ ] **Step 3: Update `HandStateMsg` and decode**

In `teleop_core/messages.py`:

```python
@dataclass(frozen=True)
class HandStateMsg:
    type: str = "hand"
    curls: tuple[float, ...] = (0.0,) * 5
    abduction: float = 0.0
    wrist_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wrist_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    valid: bool = False
    head_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    head_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    head_valid: bool = False
```

In `decode()` for `HandStateMsg`, add:

```python
head_position=tuple(float(v) for v in obj.get("head_position", (0.0, 0.0, 0.0))),
head_orientation=tuple(float(v) for v in obj.get("head_orientation", (0.0, 0.0, 0.0, 1.0))),
head_valid=bool(obj.get("head_valid", False)),
```

- [ ] **Step 4: Update `webxr_app/static/app.js` to send head pose**

Inside the animation loop, after `const ref = ...`, add:

```javascript
const viewerPose = frame.getViewerPose ? frame.getViewerPose(ref) : null;
const head = viewerPose && viewerPose.transform ? {
  position: [
    viewerPose.transform.position.x,
    viewerPose.transform.position.y,
    viewerPose.transform.position.z,
  ],
  orientation: [
    viewerPose.transform.orientation.x,
    viewerPose.transform.orientation.y,
    viewerPose.transform.orientation.z,
    viewerPose.transform.orientation.w,
  ],
  valid: true,
} : {
  position: [0, 0, 0],
  orientation: [0, 0, 0, 1],
  valid: false,
};
```

In the existing `comms.sendJson({ type: 'hand', ... })` payload, add:

```javascript
head_position: head.position,
head_orientation: head.orientation,
head_valid: head.valid,
```

- [ ] **Step 5: Verify message tests pass**

Run: `rtk python -m pytest tests/test_messages.py -q`

Expected: PASS.

---

### Task 5: Wire Telemetry Hub into TeleopServer and Add Dashboard Port

**Files:**
- Modify: `teleop_core/server.py`
- Modify: `webxr_app/__main__.py`
- Extend: `tests/test_webxr_cli_pointcloud.py`

- [ ] **Step 1: Add failing CLI/config tests**

Append to `tests/test_webxr_cli_pointcloud.py`:

```python
from teleop_core.server import ServerConfig


def test_server_config_has_dashboard_port_default():
    config = ServerConfig()

    assert config.port == 8000
    assert config.dashboard_port == 8001


def test_parse_args_accepts_dashboard_port(monkeypatch):
    from webxr_app.__main__ import _parse_args

    monkeypatch.setattr(
        sys,
        "argv",
        ["webxr_app", "--dashboard-port", "9001"],
    )

    args = _parse_args()

    assert args.dashboard_port == 9001
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `rtk python -m pytest tests/test_webxr_cli_pointcloud.py -q`

Expected: FAIL because `dashboard_port` is missing.

- [ ] **Step 3: Extend `ServerConfig`**

In `teleop_core/server.py`:

```python
dashboard_port: int = 8001
dashboard_static_dir: Path = Path(__file__).parent.parent / "webxr_app" / "dashboard_static"
urdf_path: Path | None = None
robot_assets_root: Path | None = None
dashboard_robot_hz: float = 30.0
dashboard_status_hz: float = 1.0
```

- [ ] **Step 4: Add CLI flag and pass URDF path**

In `webxr_app/__main__.py`:

```python
ap.add_argument("--dashboard-port", type=int, default=8001)
```

Resolve the effective URDF in one helper so `_make_robot_driver()` and `ServerConfig` use the same value:

```python
def _default_full_urdf() -> Path:
    return Path(__file__).resolve().parent.parent / "urdf_rc5_right_hand" / "urdf_with_simple_collisions.urdf"


def _default_floating_urdf() -> Path:
    return Path(__file__).resolve().parent.parent / "urdf_rc5_right_hand" / "robot_one_joint.urdf"
```

Use `args.effective_urdf` or a local `urdf_for_dashboard` in `main()`:

```python
urdf_for_dashboard = args.urdf or (
    _default_floating_urdf() if args.robot_backend == "floating" else _default_full_urdf()
)
```

Pass:

```python
config=ServerConfig(
    port=args.port,
    dashboard_port=args.dashboard_port,
    cert=args.cert,
    key=args.key,
    urdf_path=urdf_for_dashboard,
    robot_assets_root=urdf_for_dashboard.parent,
),
```

- [ ] **Step 5: Create telemetry hub in `TeleopServer.__init__`**

Import:

```python
import json
from .telemetry import TelemetryHub
```

Create:

```python
self._telemetry = TelemetryHub(
    point_cloud_source=self._pc,
    robot_driver=self._robot,
    workspace=self._workspace,
    urdf_url="/robot/robot.urdf",
    urdf_assets_url="/robot/assets/",
    pointcloud_hz=self._config.pointcloud_hz,
    robot_hz=self._config.dashboard_robot_hz,
    status_hz=self._config.dashboard_status_hz,
)
```

- [ ] **Step 6: Start and stop the hub in `run()`**

After `await self._robot.start()`:

```python
await self._telemetry.start()
```

In cleanup, before stopping robot and point cloud:

```python
await self._telemetry.stop()
```

- [ ] **Step 7: Replace Quest point-cloud grabbing with hub cache**

Change `_pointcloud_loop()`:

```python
last_sequence = 0
while not ws.closed:
    cloud = await self._telemetry.wait_for_pointcloud(
        after_sequence=last_sequence,
        timeout=1.0,
    )
    if cloud is None:
        continue
    last_sequence = cloud.sequence
    try:
        await ws.send_bytes(cloud.payload)
    except ConnectionResetError:
        break
```

This prevents the Quest websocket and dashboard clients from each grabbing hardware frames.

- [ ] **Step 8: Feed XR pose and anchor into hub**

In `_control_loop`, when handling `HandStateMsg`:

```python
self._latest_hand = decoded
if decoded.head_valid and decoded.valid:
    self._telemetry.update_xr_pose(
        head_position=decoded.head_position,
        head_orientation=decoded.head_orientation,
        right_wrist_position=decoded.wrist_position,
        right_wrist_orientation=decoded.wrist_orientation,
        valid=True,
        timestamp=time.monotonic(),
    )
```

In `_on_trigger`, after computing `vr_origin`:

```python
self._telemetry.update_anchor(
    (float(vr_origin[0]), float(vr_origin[1]), float(vr_origin[2])),
    timestamp=time.monotonic(),
)
```

- [ ] **Step 9: Run targeted tests**

Run: `rtk python -m pytest tests/test_webxr_cli_pointcloud.py tests/test_dashboard_telemetry.py tests/test_messages.py -q`

Expected: PASS.

---

### Task 6: Add Dashboard aiohttp App and Asset Routes

**Files:**
- Modify: `teleop_core/server.py`
- Test: `tests/test_dashboard_telemetry.py`

- [ ] **Step 1: Add safe asset path tests**

Append:

```python
from pathlib import Path

import pytest

from teleop_core.server import _resolve_robot_asset_path


def test_resolve_robot_asset_path_allows_assets_under_root(tmp_path):
    root = tmp_path / "robot"
    mesh_dir = root / "meshes"
    mesh_dir.mkdir(parents=True)
    mesh = mesh_dir / "link.stl"
    mesh.write_text("mesh")

    assert _resolve_robot_asset_path(root, "meshes/link.stl") == mesh.resolve()


def test_resolve_robot_asset_path_rejects_path_escape(tmp_path):
    root = tmp_path / "robot"
    root.mkdir()
    outside = tmp_path / "secret.stl"
    outside.write_text("secret")

    with pytest.raises(ValueError):
        _resolve_robot_asset_path(root, "../secret.stl")
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: FAIL because `_resolve_robot_asset_path` is missing.

- [ ] **Step 3: Implement dashboard route helpers in `teleop_core/server.py`**

Add helper:

```python
def _resolve_robot_asset_path(root: Path, tail: str) -> Path:
    root_resolved = Path(root).resolve()
    target = (root_resolved / tail).resolve()
    target.relative_to(root_resolved)
    return target
```

Catch `ValueError` in request handlers and return `web.Response(status=404)`.

- [ ] **Step 4: Add `_make_dashboard_app()` to `TeleopServer`**

```python
def _make_dashboard_app(self) -> web.Application:
    app = web.Application()
    static_dir = Path(self._config.dashboard_static_dir)
    app.router.add_get("/ws", self._handle_dashboard_ws)
    app.router.add_get("/api/snapshot", self._handle_dashboard_snapshot)
    app.router.add_get("/robot/robot.urdf", self._handle_robot_urdf)
    app.router.add_get("/robot/assets/{tail:.*}", self._handle_robot_asset)
    app.router.add_get("/", lambda _req: web.FileResponse(static_dir / "index.html"))
    app.router.add_static("/", path=str(static_dir), show_index=False)
    return app
```

Add handlers:

```python
async def _handle_dashboard_snapshot(self, _request) -> web.Response:
    return web.json_response(self._telemetry.snapshot())


async def _handle_robot_urdf(self, _request) -> web.StreamResponse:
    if self._config.urdf_path is None:
        return web.Response(status=404, text="URDF not configured")
    return web.FileResponse(Path(self._config.urdf_path))


async def _handle_robot_asset(self, request) -> web.StreamResponse:
    root = self._config.robot_assets_root
    if root is None:
        return web.Response(status=404, text="robot asset root not configured")
    try:
        path = _resolve_robot_asset_path(Path(root), request.match_info["tail"])
    except ValueError:
        return web.Response(status=404)
    if not path.exists() or not path.is_file():
        return web.Response(status=404)
    return web.FileResponse(path)
```

Add websocket:

```python
async def _handle_dashboard_ws(self, request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    await ws.send_str(json.dumps(self._telemetry.snapshot()))

    async def json_loop() -> None:
        # Snapshot JSON is small and carries robot/XR state, so send it at
        # the dashboard robot rate. The status panel can still display
        # lower-frequency summaries from the latest snapshot fields.
        period = 1.0 / max(self._config.dashboard_robot_hz, 1e-3)
        while not ws.closed:
            await ws.send_str(json.dumps(self._telemetry.snapshot()))
            await asyncio.sleep(period)

    async def cloud_loop() -> None:
        last_sequence = 0
        while not ws.closed:
            cloud = await self._telemetry.wait_for_pointcloud(
                after_sequence=last_sequence,
                timeout=1.0,
            )
            if cloud is None:
                continue
            last_sequence = cloud.sequence
            await ws.send_bytes(cloud.payload)

    tasks = [
        asyncio.create_task(json_loop(), name="dashboard_json_loop"),
        asyncio.create_task(cloud_loop(), name="dashboard_cloud_loop"),
    ]
    try:
        async for msg in ws:
            if msg.type == WSMsgType.ERROR:
                break
            if msg.type == WSMsgType.TEXT:
                await ws.send_str(json.dumps({
                    "type": "error",
                    "message": "dashboard is read-only",
                }))
    finally:
        for task in tasks:
            task.cancel()
        for task in tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task
    return ws
```

- [ ] **Step 5: Start both sites in `run()`**

Refactor app creation:

```python
teleop_app = self._make_teleop_app()
dashboard_app = self._make_dashboard_app()
teleop_runner = web.AppRunner(teleop_app)
dashboard_runner = web.AppRunner(dashboard_app)
await teleop_runner.setup()
await dashboard_runner.setup()
teleop_site = web.TCPSite(...)
dashboard_site = web.TCPSite(
    dashboard_runner,
    self._config.host,
    self._config.dashboard_port,
    ssl_context=ssl_context,
)
```

Start both and print both URLs. Cleanup both runners.

- [ ] **Step 6: Verify route helper tests**

Run: `rtk python -m pytest tests/test_dashboard_telemetry.py -q`

Expected: PASS.

---

### Task 7: Build Dashboard Frontend

**Files:**
- Create: `webxr_app/dashboard_static/index.html`
- Create: `webxr_app/dashboard_static/style.css`
- Create: `webxr_app/dashboard_static/dashboard.js`
- Create: `webxr_app/dashboard_static/modules/dashboard_comms.js`
- Create: `webxr_app/dashboard_static/modules/dashboard_scene.js`
- Create: `webxr_app/dashboard_static/modules/robot_view.js`
- Create: `webxr_app/dashboard_static/modules/dashboard_pointcloud_view.js`
- Create: `webxr_app/dashboard_static/modules/workspace_layer.js`
- Create: `webxr_app/dashboard_static/modules/xr_markers.js`
- Create: `webxr_app/dashboard_static/modules/status_panel.js`

- [ ] **Step 1: Create HTML shell**

`webxr_app/dashboard_static/index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>VR setup dashboard</title>
  <link rel="stylesheet" href="./style.css">
  <script type="importmap">
  {
    "imports": {
      "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
      "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/",
      "urdf-loader": "https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/src/URDFLoader.js"
    }
  }
  </script>
</head>
<body>
  <main id="app">
    <section id="viewport"></section>
    <aside id="panel">
      <div class="panel-section">
        <h1>Setup Dashboard</h1>
        <div id="connection">Connecting</div>
      </div>
      <div class="panel-section">
        <h2>Layers</h2>
        <label><input id="toggle-robot" type="checkbox" checked> Robot</label>
        <label><input id="toggle-cloud" type="checkbox" checked> Point cloud</label>
        <label><input id="toggle-workspace" type="checkbox" checked> Workspace</label>
        <label><input id="toggle-xr" type="checkbox" checked> XR head/wrist</label>
      </div>
      <div id="status"></div>
    </aside>
  </main>
  <script type="module" src="./dashboard.js"></script>
</body>
</html>
```

- [ ] **Step 2: Create CSS**

Use restrained desktop tooling CSS: full viewport, dark neutral canvas, right panel about `320px`, compact labels, no decorative gradients.

- [ ] **Step 3: Create dashboard comms module**

`dashboard_comms.js` mirrors the existing `Comms` class but defaults to `/ws` and exposes `onJson`, `onBinary`, and connection-state callbacks.

- [ ] **Step 4: Create scene module**

`dashboard_scene.js` creates:

```javascript
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function robotToThreeVector(v) {
  return new THREE.Vector3(v[0], v[2], -v[1]);
}

export class DashboardScene {
  constructor(container) {
    this.scene = new THREE.Scene();
    this.world = new THREE.Group();
    this.scene.add(this.world);
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.01, 50);
    this.camera.position.set(1.2, 1.0, 1.6);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.3, 0.3, 0.0);
    this.controls.update();
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x202020, 2.0));
    const grid = new THREE.GridHelper(2.0, 20, 0x557788, 0x334455);
    this.world.add(grid);
    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.renderer.setAnimationLoop(() => this.renderer.render(this.scene, this.camera));
  }

  resize() {
    const rect = this.renderer.domElement.parentElement.getBoundingClientRect();
    this.camera.aspect = rect.width / Math.max(1, rect.height);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(rect.width, rect.height, false);
  }
}
```

- [ ] **Step 5: Create robot view module**

`robot_view.js` loads the URDF and applies named joints:

```javascript
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

export class RobotView {
  constructor(world) {
    this.group = new THREE.Group();
    this.group.rotation.x = -Math.PI / 2;
    this.robot = null;
    world.add(this.group);
  }

  load(model) {
    const loader = new URDFLoader();
    loader.workingPath = model.urdf_assets_url;
    loader.load(model.urdf_url, robot => {
      this.group.clear();
      this.robot = robot;
      robot.traverse(obj => {
        if (obj.isMesh && obj.material) {
          obj.material.side = THREE.DoubleSide;
        }
      });
      this.group.add(robot);
    });
  }

  applyJoints(joints) {
    if (!this.robot || !joints) return;
    for (const [name, value] of Object.entries(joints)) {
      if (this.robot.joints && this.robot.joints[name]) {
        this.robot.setJointValue(name, value);
      }
    }
  }

  setVisible(visible) {
    this.group.visible = !!visible;
  }
}
```

- [ ] **Step 6: Create point cloud, workspace, and XR marker modules**

`dashboard_pointcloud_view.js` should reuse the current binary layout and transform each decoded point through `robotToThreeVector`.

`workspace_layer.js` should draw an edge box from `snapshot.workspace.min/max` in robot coordinates transformed to Three coordinates.

`xr_markers.js` should:

```javascript
function helmetToRobotPosition(position, origin) {
  const dx = position[0] - origin[0];
  const dy = position[1] - origin[1];
  const dz = position[2] - origin[2];
  return [dx, -dz, dy];
}
```

Render a head sphere and right-wrist sphere only when `snapshot.xr.aligned === true`. If unaligned, hide the markers and show `XR unaligned` in the status panel.

- [ ] **Step 7: Create status panel module**

Show connection state, URDF URL, robot joint count, cloud point count/sequence, XR aligned status, and any robot/pointcloud error strings.

- [ ] **Step 8: Create `dashboard.js`**

Wire modules:

```javascript
const scene = new DashboardScene(document.getElementById('viewport'));
const robot = new RobotView(scene.world);
const cloud = new DashboardPointCloudView(scene.world);
const workspace = new WorkspaceLayer(scene.world);
const xr = new XRMarkers(scene.world);
const status = new StatusPanel(document.getElementById('status'), document.getElementById('connection'));
const comms = new DashboardComms('/ws');

let modelLoaded = false;
comms.onJson = msg => {
  if (msg.type !== 'snapshot') return;
  if (!modelLoaded) {
    robot.load(msg.model);
    workspace.setBounds(msg.workspace.min, msg.workspace.max);
    modelLoaded = true;
  }
  robot.applyJoints(msg.robot.joints);
  xr.update(msg.xr);
  status.update(msg);
};
comms.onBinary = buf => cloud.ingest(buf);
```

Add layer toggle listeners that call `setVisible(...)`.

- [ ] **Step 9: Run Python tests**

Run: `rtk python -m pytest -q`

Expected: PASS. Frontend will be verified in Task 8.

---

### Task 8: Smoke Test Servers and Dashboard Rendering

**Files:**
- No planned code changes unless verification finds a defect.

- [ ] **Step 1: Start the app with mock backends**

Run:

```bash
rtk python -m webxr_app --pc-backend mock --robot-backend pybullet --port 8000 --dashboard-port 8001
```

Expected terminal output includes:

```text
[teleop] serving on http://0.0.0.0:8000
[teleop] dashboard on http://0.0.0.0:8001
```

- [ ] **Step 2: Open dashboard in the browser**

Use the Browser plugin to open:

```text
http://localhost:8001
```

Expected:
- Canvas is nonblank.
- Right panel says connected.
- Robot layer loads without console errors.
- Point cloud layer shows the mock cloud.
- Workspace layer is visible.
- XR markers are hidden and status says unaligned before Quest engage.

- [ ] **Step 3: Verify HTTP endpoints**

Run:

```bash
rtk curl -I http://localhost:8001/
rtk curl -I http://localhost:8001/robot/robot.urdf
rtk curl http://localhost:8001/api/snapshot
```

Expected:
- `/` returns HTTP 200.
- `/robot/robot.urdf` returns HTTP 200.
- `/api/snapshot` JSON has `type: "snapshot"`, `model`, `workspace`, `robot`, `pointcloud`, `xr`, and `status`.

- [ ] **Step 4: Run full regression tests**

Run:

```bash
rtk python -m pytest -q
```

Expected: PASS.

- [ ] **Step 5: Stop the dev server**

Stop the running server session cleanly with Ctrl-C or `rtk kill <pid>` if it was started in a separate shell session.

---

### Task 9: Documentation Update

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add dashboard CLI documentation**

Update the CLI flags table:

```markdown
| `--dashboard-port` | `8001` | Read-only desktop setup dashboard port |
```

Add a short section after Quick Start:

```markdown
### Setup dashboard

The server also starts a read-only desktop dashboard on
`http://<host>:8001` by default. It renders the configured URDF,
the fused point cloud, workspace bounds, and Quest head/right-wrist
markers after the operator engages tracking once. The dashboard is
observability-only in v1: it does not write config files or command
robot motion.
```

- [ ] **Step 2: Run tests**

Run: `rtk python -m pytest -q`

Expected: PASS.

---

## Final Verification Checklist

- [ ] `rtk git status --short` shows only intended files changed.
- [ ] `rtk python -m pytest -q` passes.
- [ ] `python -m webxr_app --pc-backend mock --robot-backend pybullet --port 8000 --dashboard-port 8001` starts both sites.
- [ ] Browser smoke test confirms the dashboard canvas is nonblank and no JS module import errors appear.
- [ ] `http://localhost:8001/api/snapshot` returns complete snapshot JSON.
- [ ] Multiple dashboard clients do not cause multiple `PointCloudSource.grab()` calls per frame; the hub test proves fanout over cached payloads.

## Execution Notes

- Keep dashboard write paths absent in v1. If a dashboard client sends text over `/ws`, reply with a read-only error and do not mutate state.
- Do not move safety-monitor work into this change. Render current workspace bounds and current client-side warning state only.
- Do not add npm/Vite. Use import maps and CDN modules matching the existing WebXR app style.
- Preserve the dependency direction: `teleop_core` may serve paths from config, but it must not import `webxr_app` or `teleop_backends`.
