"""Dry-run robot driver: logs commands, never moves anything.

Useful for headless development and CI. Reports a fixed home pose and
echoes the last commanded target as its "current" state, so the safety
monitor doesn't false-positive on lag.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from teleop_core.robot import RobotCommand, RobotDriver, RobotState
from teleop_core.types import Pose


class NoopRobotDriver(RobotDriver):
    """Doesn't drive anything; just records the last command."""

    def __init__(self, home: Pose) -> None:
        self._home = home
        self._last_cmd: Optional[RobotCommand] = None

    async def start(self) -> None:
        pass

    async def stop(self) -> None:
        pass

    async def send(self, cmd: RobotCommand) -> None:
        self._last_cmd = cmd

    async def get_state(self) -> RobotState:
        # Echo the last commanded target so the safety monitor's lag
        # detector treats us as a perfectly-tracking robot.
        pose = self._home
        curls = np.zeros(5, dtype=np.float32)
        if self._last_cmd is not None:
            if self._last_cmd.target_wrist_pose is not None:
                pose = self._last_cmd.target_wrist_pose
            if self._last_cmd.target_finger_curls is not None:
                curls = self._last_cmd.target_finger_curls
        return RobotState(
            wrist_pose=pose,
            joint_angles=np.zeros(0, dtype=np.float32),
            finger_curls=curls,
            timestamp=time.monotonic(),
        )

    @property
    def home_pose(self) -> Pose:
        return self._home
