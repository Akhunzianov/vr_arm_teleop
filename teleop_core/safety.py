"""Operator/robot safety monitor.

Watches the difference between what we *commanded* the robot to do and
what it's *actually doing*, plus user-wrist motion vs the workspace.

Emits :class:`SafetyEvent` items that the server forwards to the browser
as warning prompts. The orchestrator may also use the events to pause
the tracker (e.g. on persistent lag).

Reasons we trigger:
- ``OUT_OF_WORKSPACE``: operator's raw target left the box.
- ``LAGGING``: robot is far behind the commanded pose for too long.
- ``ROBOT_STALE``: no fresh state from the robot driver.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from .robot import RobotState
from .tracking import TrackingResult


class SafetyKind(Enum):
    OUT_OF_WORKSPACE = "out_of_workspace"
    LAGGING = "lagging"
    ROBOT_STALE = "robot_stale"


class Severity(Enum):
    OK = "ok"
    WARN = "warn"
    ERROR = "error"


@dataclass(frozen=True)
class SafetyEvent:
    """One discrete event the server should display / log / react to."""
    kind: SafetyKind
    severity: Severity
    message: str         # human-readable, shown in VR
    timestamp: float


@dataclass
class SafetyConfig:
    """Tunables for the safety monitor."""
    max_lag_meters: float = 0.05           # commanded vs actual position
    lag_grace_frames: int = 10             # tolerate brief lag before warning
    state_staleness_seconds: float = 0.5   # threshold for ROBOT_STALE


class SafetyMonitor:
    """Stateful per-frame safety checks."""

    def __init__(self, config: SafetyConfig) -> None:
        self._config = config
        self._lag_frames = 0

    def step(
        self,
        tracking: Optional[TrackingResult],
        robot: RobotState,
        now: float,
    ) -> tuple[SafetyEvent, ...]:
        """Run all checks; return any events that fired this tick."""
        raise NotImplementedError
