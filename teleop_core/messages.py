"""WebSocket message types (control channel only).

The control channel is JSON. Each message has a ``type`` discriminator
matching one of the dataclasses below; ``encode`` / ``decode`` helpers
produce / parse the JSON strings the server and client exchange.

The point cloud uses a **separate binary channel** -- see
:mod:`teleop_core.point_cloud` for that wire format.

Keep this file free of any business logic so the same definitions can
be referenced from the frontend (the structure is mirrored in JS).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, is_dataclass


# ----- Client -> Server ---------------------------------------------------

@dataclass(frozen=True)
class HandStateMsg:
    """Per-frame right-hand state. Streamed at ~30 Hz."""
    type: str = "hand"
    curls: tuple[float, ...] = (0.0,) * 5   # thumb..little, 0..1
    abduction: float = 0.0                  # raw radians (server normalizes)
    wrist_position: tuple[float, float, float] = (0.0, 0.0, 0.0)  # world, m
    wrist_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    valid: bool = False
    head_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    head_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    head_valid: bool = False


@dataclass(frozen=True)
class ButtonMsg:
    """Edge event for a digital button. Sent only on rising edge."""
    type: str = "button"
    hand: str = "left"        # 'left' | 'right'
    name: str = "x_click"     # 'x_click', 'y_click', 'a_click', 'b_click', 'menu_click', 'trigger', 'grip', 'thumbstick'
    pressed: bool = True


@dataclass(frozen=True)
class TriggerMsg:
    """Analog trigger / grip value. Streamed when changing."""
    type: str = "trigger"
    hand: str = "left"
    name: str = "trigger"     # 'trigger' | 'grip'
    value: float = 0.0        # 0..1


# ----- Server -> Client ---------------------------------------------------

@dataclass(frozen=True)
class PhaseMsg:
    """Tell the client which phase we're in, drives the UI."""
    type: str = "phase"
    phase: str = "idle"
    # 'idle' | 'finger_cal' | 'ready' | 'tracking' | 'fault'


@dataclass(frozen=True)
class PromptMsg:
    """Head-locked text panel content."""
    type: str = "prompt"
    text: str | None = None
    severity: str = "info"    # 'info' | 'warn' | 'error'


@dataclass(frozen=True)
class WorkspaceMsg:
    """One-time announcement of the workspace box so the client can draw it."""
    type: str = "workspace"
    min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    max: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class AnchorMsg:
    """Mapping from robot-world coordinates to the VR play_space frame.

    Sent when the operator engages tracking. The client renders
    robot-world geometry (e.g. the workspace box) at
    ``vr_position_of_robot_origin + robot_world_point``. The mapping
    matches the tracker, which uses pure world-frame translation
    deltas (so the play_space axes are assumed aligned with the robot
    world axes; only the origin offset changes).
    """
    type: str = "anchor"
    vr_position_of_robot_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RobotEchoMsg:
    """Live echo of robot state for HUD/debug overlay."""
    type: str = "robot"
    wrist_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wrist_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    finger_curls: tuple[float, ...] = (0.0,) * 5
    timestamp: float = 0.0


@dataclass(frozen=True)
class SafetyMsg:
    """A safety event the client should surface (warning panel, color, etc.)."""
    type: str = "safety"
    kind: str = ""            # SafetyKind value
    severity: str = "warn"    # Severity value
    message: str = ""


# Mapping from the wire ``type`` discriminator to the inbound dataclass
# the server reconstructs. Only Client->Server messages live here; outbound
# messages are dataclasses we encode but never decode.
_CLIENT_TYPES = {
    "hand": HandStateMsg,
    "button": ButtonMsg,
    "trigger": TriggerMsg,
}


def encode(msg) -> str:
    """Serialize any message dataclass into the JSON the client expects."""
    if not is_dataclass(msg):
        raise TypeError(f"encode expects a dataclass, got {type(msg).__name__}")
    return json.dumps(asdict(msg))


def decode(text: str):
    """Parse incoming JSON into one of the Client->Server dataclasses."""
    obj = json.loads(text)
    t = obj.get("type")
    cls = _CLIENT_TYPES.get(t)
    if cls is None:
        raise ValueError(f"unknown control message type: {t!r}")
    if cls is HandStateMsg:
        return HandStateMsg(
            curls=tuple(float(v) for v in obj.get("curls", (0.0,) * 5)),
            abduction=float(obj.get("abduction", 0.0)),
            wrist_position=tuple(float(v) for v in obj.get("wrist_position", (0.0, 0.0, 0.0))),
            wrist_orientation=tuple(float(v) for v in obj.get("wrist_orientation", (0.0, 0.0, 0.0, 1.0))),
            valid=bool(obj.get("valid", False)),
            head_position=tuple(float(v) for v in obj.get("head_position", (0.0, 0.0, 0.0))),
            head_orientation=tuple(float(v) for v in obj.get("head_orientation", (0.0, 0.0, 0.0, 1.0))),
            head_valid=bool(obj.get("head_valid", False)),
        )
    if cls is ButtonMsg:
        return ButtonMsg(
            hand=str(obj.get("hand", "left")),
            name=str(obj.get("name", "x_click")),
            pressed=bool(obj.get("pressed", True)),
        )
    if cls is TriggerMsg:
        return TriggerMsg(
            hand=str(obj.get("hand", "left")),
            name=str(obj.get("name", "trigger")),
            value=float(obj.get("value", 0.0)),
        )
    raise ValueError(f"unhandled control message type: {t!r}")
