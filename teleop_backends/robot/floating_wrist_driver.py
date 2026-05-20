"""Pybullet driver for a free-floating wrist (no arm, no IK).

Loads ``robot_one_joint.urdf`` -- a floating base joint that carries the
hand. Send-target wrist poses are applied directly with
``resetBasePositionAndOrientation``; the wrist follows commands 1:1 so
this is the right backend for isolating "is the tracker math correct?"
from "is the arm IK any good?".

Finger curls work the same way as in the full PybulletRobotDriver -- one
normalised scalar per finger drives all of that finger's coupled joints.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import pybullet as p

from teleop_core.robot import RobotCommand, RobotDriver, RobotState
from teleop_core.types import Pose


_DEFAULT_FINGER_JOINT_GROUPS: dict[int, tuple[str, ...]] = {
    0: ("right_thumb_cmc_flex", "right_thumb_mcp", "right_thumb_ip"),
    1: ("right_index_mcp_flex", "right_index_pip", "right_index_dip"),
    2: ("right_middle_mcp_flex", "right_middle_pip", "right_middle_dip"),
    3: ("right_ring_mcp_flex", "right_ring_pip", "right_ring_dip"),
    4: ("right_pinky_mcp_flex", "right_pinky_pip", "right_pinky_dip"),
}

_THUMB_OPPOSITION_JOINT = "right_thumb_cmc_abd"
_THUMB_OPPOSITION_DEFAULT_RAD = 0.5
# URDF allows [0, 1.745] rad (100°) but realistic CMC abduction range is
# closer to ~50–60°. Map normalised abduction into this narrower band so
# typical user thumb motion doesn't saturate the joint.
_THUMB_ABD_MIN_RAD = 0.2
_THUMB_ABD_MAX_RAD = 0.85

_DEFAULT_EE_LINK_NAME = "right_tcp_link"
_DEFAULT_HOME_POS = (0.3, 0.3, 0.3)
_DEFAULT_HOME_ORI = (0.0, 0.7071068, 0.7071068, 0.0)


@dataclass(frozen=True)
class _JointInfo:
    index: int
    lower: float
    upper: float


class FloatingWristDriver(RobotDriver):
    """Teleports a hand-only URDF directly to commanded wrist poses."""

    def __init__(
        self,
        urdf_path: Path,
        ee_link_name: str = _DEFAULT_EE_LINK_NAME,
        finger_joint_groups: dict[int, tuple[str, ...]] | None = None,
        home_position: tuple[float, float, float] = _DEFAULT_HOME_POS,
        home_orientation: tuple[float, float, float, float] = _DEFAULT_HOME_ORI,
        client_id: int | None = None,
        gui: bool = False,
        sim_hz: float = 240.0,
    ) -> None:
        self._urdf_path = Path(urdf_path)
        self._ee_link_name = ee_link_name
        self._finger_groups = dict(finger_joint_groups or _DEFAULT_FINGER_JOINT_GROUPS)
        self._home_position = tuple(float(v) for v in home_position)
        self._home_orientation = tuple(float(v) for v in home_orientation)
        self._client_id: Optional[int] = client_id
        self._gui = gui
        self._sim_dt = 1.0 / float(sim_hz)
        self._owns_client = client_id is None

        self._body_id: Optional[int] = None
        self._finger_joints: dict[int, list[_JointInfo]] = {}
        self._thumb_opp: Optional[_JointInfo] = None
        self._ee_link_index: int = -1

        self._lock = asyncio.Lock()
        self._step_task: Optional[asyncio.Task] = None
        self._stopping = False

        self._home_pose: Optional[Pose] = None
        self._last_curls = np.zeros(5, dtype=np.float32)

        # right_tcp_link sits 0.065 m along +Z of right_base_link in the
        # URDF. Cached so we don't have to query pybullet each tick.
        self._tcp_local_offset = np.array([0.0, 0.0, 0.065], dtype=np.float64)

    # --- lifecycle -----------------------------------------------------

    async def start(self) -> None:
        if self._body_id is not None:
            return

        def _connect_and_load() -> None:
            if self._owns_client:
                mode = p.GUI if self._gui else p.DIRECT
                self._client_id = p.connect(mode)
            cid = self._client_id

            # No gravity -- a floating base would just fall. We are the
            # sole authority on where the wrist is.
            p.setGravity(0, 0, 0, physicsClientId=cid)
            p.setTimeStep(self._sim_dt, physicsClientId=cid)
            p.setRealTimeSimulation(0, physicsClientId=cid)

            self._body_id = p.loadURDF(
                str(self._urdf_path),
                basePosition=self._home_position,
                baseOrientation=self._home_orientation,
                useFixedBase=False,
                physicsClientId=cid,
                flags=p.URDF_USE_INERTIA_FROM_FILE,
            )

            joint_by_name: dict[str, _JointInfo] = {}
            link_index_by_name: dict[str, int] = {}
            n_joints = p.getNumJoints(self._body_id, physicsClientId=cid)
            for i in range(n_joints):
                info = p.getJointInfo(self._body_id, i, physicsClientId=cid)
                name = info[1].decode()
                lower, upper = info[8], info[9]
                child_link_name = info[12].decode()
                joint_by_name[name] = _JointInfo(index=i, lower=lower, upper=upper)
                link_index_by_name[child_link_name] = i

            self._finger_joints = {
                f: [joint_by_name[n] for n in names]
                for f, names in self._finger_groups.items()
            }
            self._thumb_opp = joint_by_name[_THUMB_OPPOSITION_JOINT]
            self._ee_link_index = link_index_by_name[self._ee_link_name]

            # Hold the base where we want it, freeze fingers open.
            p.resetBasePositionAndOrientation(
                self._body_id, self._home_position, self._home_orientation,
                physicsClientId=cid,
            )
            p.resetBaseVelocity(self._body_id, [0, 0, 0], [0, 0, 0],
                                physicsClientId=cid)
            p.resetJointState(
                self._body_id, self._thumb_opp.index,
                _THUMB_OPPOSITION_DEFAULT_RAD, physicsClientId=cid,
            )
            self._motor_target(self._thumb_opp, _THUMB_OPPOSITION_DEFAULT_RAD)
            for joints in self._finger_joints.values():
                for j in joints:
                    self._motor_target(j, j.lower)

            p.stepSimulation(physicsClientId=cid)
            pos, orn = self._read_ee_pose()
            self._home_pose = Pose(
                position=np.asarray(pos, dtype=np.float64),
                orientation=np.asarray(orn, dtype=np.float64),
                frame="world",
            )

        await asyncio.to_thread(_connect_and_load)
        self._stopping = False
        self._step_task = asyncio.create_task(self._step_loop())

    async def stop(self) -> None:
        self._stopping = True
        if self._step_task is not None:
            self._step_task.cancel()
            try:
                await self._step_task
            except (asyncio.CancelledError, Exception):
                pass
            self._step_task = None
        if self._owns_client and self._client_id is not None:
            cid = self._client_id
            await asyncio.to_thread(p.disconnect, physicsClientId=cid)
        self._body_id = None
        self._client_id = None if self._owns_client else self._client_id

    # --- command + state ----------------------------------------------

    async def send(self, cmd: RobotCommand) -> None:
        if self._body_id is None:
            raise RuntimeError("FloatingWristDriver.send called before start()")
        target_pose = cmd.target_wrist_pose
        target_curls = cmd.target_finger_curls
        target_abd = cmd.target_thumb_abduction

        def _apply() -> None:
            if target_pose is not None:
                # The TCP we care about (the "wrist" from the tracker's POV)
                # is offset from the URDF's base link by a fixed local
                # vector; subtract that offset (rotated into world by the
                # commanded orientation) so the TCP, not the base, lands
                # exactly on the requested pose.
                base_pos, base_orn = self._target_pose_to_base(target_pose)
                p.resetBasePositionAndOrientation(
                    self._body_id, base_pos, base_orn,
                    physicsClientId=self._client_id,
                )
                # Wipe velocities so the kinematic teleport doesn't leak
                # into Bullet's integrator on the next step.
                p.resetBaseVelocity(
                    self._body_id, [0, 0, 0], [0, 0, 0],
                    physicsClientId=self._client_id,
                )
            if target_curls is not None:
                curls = np.asarray(target_curls, dtype=np.float32).reshape(-1)
                if curls.shape[0] != 5:
                    raise ValueError("target_finger_curls must have length 5")
                for finger, c in enumerate(curls):
                    self._set_finger_curl(finger, float(c))
                self._last_curls = curls.astype(np.float32)
            if target_abd is not None and self._thumb_opp is not None:
                a = float(np.clip(target_abd, 0.0, 1.0))
                lo = max(self._thumb_opp.lower, _THUMB_ABD_MIN_RAD)
                hi = min(self._thumb_opp.upper, _THUMB_ABD_MAX_RAD)
                rad = lo + a * (hi - lo)
                self._motor_target(self._thumb_opp, rad)

        async with self._lock:
            await asyncio.to_thread(_apply)

    async def get_state(self) -> RobotState:
        if self._body_id is None:
            raise RuntimeError("FloatingWristDriver.get_state called before start()")

        def _read() -> RobotState:
            pos, orn = self._read_ee_pose()
            return RobotState(
                wrist_pose=Pose(
                    position=np.asarray(pos, dtype=np.float64),
                    orientation=np.asarray(orn, dtype=np.float64),
                    frame="world",
                ),
                joint_angles=np.zeros(0, dtype=np.float32),
                finger_curls=self._last_curls.copy(),
                timestamp=time.monotonic(),
            )

        async with self._lock:
            return await asyncio.to_thread(_read)

    @property
    def home_pose(self) -> Pose:
        if self._home_pose is None:
            raise RuntimeError("home_pose available only after start()")
        return self._home_pose

    # --- internals -----------------------------------------------------

    async def _step_loop(self) -> None:
        next_t = time.monotonic()
        while not self._stopping:
            next_t += self._sim_dt
            async with self._lock:
                await asyncio.to_thread(
                    p.stepSimulation, physicsClientId=self._client_id
                )
            sleep = next_t - time.monotonic()
            if sleep > 0:
                await asyncio.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _target_pose_to_base(self, target: Pose) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Convert a desired TCP pose to the URDF's base position/orientation."""
        orn = tuple(float(v) for v in target.orientation)
        # Rotate the local TCP offset into world by the target orientation.
        rotated = p.rotateVector(orn, self._tcp_local_offset.tolist())
        base = np.asarray(target.position, dtype=np.float64) - np.asarray(rotated)
        return tuple(float(v) for v in base), orn

    def _read_ee_pose(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        state = p.getLinkState(
            self._body_id, self._ee_link_index, physicsClientId=self._client_id
        )
        return state[4], state[5]

    def _set_finger_curl(self, finger: int, curl: float) -> None:
        c = float(np.clip(curl, 0.0, 1.0))
        joints = self._finger_joints.get(finger)
        if not joints:
            return
        for joint in joints:
            target = joint.lower + c * (joint.upper - joint.lower)
            self._motor_target(joint, target)

    def _motor_target(self, joint: _JointInfo, target: float) -> None:
        if joint.upper > joint.lower:
            target = float(np.clip(target, joint.lower, joint.upper))
        p.setJointMotorControl2(
            self._body_id, joint.index, p.POSITION_CONTROL,
            targetPosition=float(target),
            physicsClientId=self._client_id,
        )
