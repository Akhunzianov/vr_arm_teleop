"""Workspace volume + safety geometry.

The workspace is the axis-aligned box in world coordinates that the
robot is allowed to move within. The tracker clamps every commanded
pose against it; the safety layer raises a warning when the operator's
wrist exits it.

Keeping this independent of the robot/tracker makes it trivial to:
- Render the box in VR for the operator (we serialize it to the client).
- Visualise it offline (matplotlib, blender, etc.) for setup.
- Unit-test clamping logic without spinning up anything.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Workspace:
    """Axis-aligned box in the world frame."""

    min_corner: np.ndarray   # shape (3,), meters
    max_corner: np.ndarray   # shape (3,), meters
    frame: str = "world"

    def contains(self, point: np.ndarray) -> bool:
        """True iff ``point`` is inside the box (boundary inclusive)."""
        p = np.asarray(point)
        return bool(np.all(p >= self.min_corner) and np.all(p <= self.max_corner))

    def clamp(self, point: np.ndarray) -> tuple[np.ndarray, bool]:
        """Clamp ``point`` into the box.

        Returns
        -------
        clamped : np.ndarray
            The closest point inside the box (component-wise clamp).
        was_outside : bool
            True iff the input was strictly outside; useful for triggering
            "out of workspace" warnings without re-comparing afterwards.
        """
        p = np.asarray(point)
        clamped = np.minimum(np.maximum(p, self.min_corner), self.max_corner)
        was_outside = bool(np.any(p < self.min_corner) or np.any(p > self.max_corner))
        return clamped, was_outside

    def as_dict(self) -> dict:
        """JSON-friendly representation, for sending to the client."""
        return {
            "min": [float(v) for v in self.min_corner],
            "max": [float(v) for v in self.max_corner],
            "frame": self.frame,
        }
