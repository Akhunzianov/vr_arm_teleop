"""Real-hardware driver: Aero hand + (whatever arm we end up with).

Composes two sub-drivers:
- the Aero hand SDK for fingers (we already use this in the SteamVR path),
- a separate arm SDK / ROS2 client / serial protocol for the arm.

Finger curls flow to the hand; wrist pose flows to the arm. State
comes back from both and is merged into a single :class:`RobotState`.

Left as a stub until the actual arm hardware is wired in.
"""

from __future__ import annotations

from typing import Optional

from teleop_core.robot import RobotCommand, RobotDriver, RobotState
from teleop_core.types import Pose


class AeroArmDriver(RobotDriver):
    """Composite: real Aero hand + real arm. Stub for now."""

    def __init__(
        self,
        aero_serial_port: Optional[str] = None,
        arm_endpoint: Optional[str] = None,   # placeholder; depends on the arm
    ) -> None:
        self._aero_port = aero_serial_port
        self._arm_endpoint = arm_endpoint

    async def start(self) -> None:
        raise NotImplementedError

    async def stop(self) -> None:
        raise NotImplementedError

    async def send(self, cmd: RobotCommand) -> None:
        raise NotImplementedError

    async def get_state(self) -> RobotState:
        raise NotImplementedError

    @property
    def home_pose(self) -> Pose:
        raise NotImplementedError
