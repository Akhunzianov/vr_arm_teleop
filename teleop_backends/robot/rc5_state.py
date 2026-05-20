"""RC5 live state helpers.

The RC5 SDK is optional in this repository. Import it lazily so tests and
simulation backends still work on machines that do not have the robot API
installed.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from typing import Protocol

import numpy as np

from teleop_backends.camera_calibration import JointStateProvider


RC5_ARM_JOINT_NAMES: tuple[str, ...] = (
    "joint0",
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
)


class _RobotApiFactory(Protocol):
    def __call__(
        self,
        *,
        ip: str,
        read_only: bool,
        show_std_traceback: bool,
    ): ...


def load_rc5_robot_api(rc5_api_path: Path | str | None = None):
    """Return the SDK ``RobotApi`` class, adding a source path when provided."""

    path = Path(rc5_api_path) if rc5_api_path is not None else None
    env_path = os.environ.get("RC5_API_PATH")
    if path is None and env_path:
        path = Path(env_path)
    if path is not None:
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)
    try:
        module = importlib.import_module("API.rc_api")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "RC5 Python SDK is not importable. Install python_api_1.4.2.zip "
            "or provide --rc5-api-path / RC5_API_PATH pointing at the SDK root."
        ) from exc
    return module.RobotApi


def read_rc5_named_joint_angles(robot) -> dict[str, float]:
    """Read live RC5 arm joints as URDF-name keyed radians."""

    raw = robot.motion.joint.get_actual_position(units="rad")
    angles = np.asarray(raw, dtype=np.float64).reshape(-1)
    if angles.shape != (len(RC5_ARM_JOINT_NAMES),):
        raise ValueError(
            f"expected 6 RC5 joint angles, got {int(angles.shape[0])}"
        )
    if not np.all(np.isfinite(angles)):
        raise ValueError("RC5 joint state contains non-finite values")
    return {
        name: float(angle)
        for name, angle in zip(RC5_ARM_JOINT_NAMES, angles)
    }


class RC5JointStateReader(JointStateProvider):
    """One-shot read-only RC5 joint-state provider for calibration."""

    def __init__(
        self,
        *,
        arm_ip: str,
        rc5_api_path: Path | str | None = None,
        robot_api_factory: _RobotApiFactory | None = None,
    ) -> None:
        self._arm_ip = arm_ip
        self._rc5_api_path = rc5_api_path
        self._robot_api_factory = robot_api_factory

    def read_joint_state(self) -> dict[str, float]:
        factory = self._robot_api_factory
        if factory is None:
            factory = load_rc5_robot_api(self._rc5_api_path)
        robot = factory(
            ip=self._arm_ip,
            read_only=True,
            show_std_traceback=True,
        )
        try:
            return read_rc5_named_joint_angles(robot)
        finally:
            disconnect = getattr(robot, "disconnect", None)
            if disconnect is not None:
                disconnect()
