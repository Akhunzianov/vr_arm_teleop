"""Pose, transforms, and a couple of shared dataclasses.

Pure data types only. No I/O, no async, no third-party dependencies beyond
numpy. Anything that needs to talk to the network, GPU, or hardware lives
in the corresponding backend or in server.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose:
    """6-DoF pose in some named coordinate frame.

    Attributes
    ----------
    position : np.ndarray
        Shape (3,), meters.
    orientation : np.ndarray
        Shape (4,), quaternion in (x, y, z, w) order.
    frame : str
        Name of the reference frame (e.g. 'world', 'robot_base',
        'play_space'). Pure annotation -- the math doesn't enforce
        anything; it just helps catch frame-mismatch bugs in code review.
    """

    position: np.ndarray
    orientation: np.ndarray
    frame: str = "world"

    @staticmethod
    def identity(frame: str = "world") -> "Pose":
        return Pose(
            position=np.zeros(3, dtype=np.float64),
            orientation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            frame=frame,
        )

    def translated(self, delta: np.ndarray) -> "Pose":
        """Return a new Pose with ``position`` shifted by ``delta``."""
        return Pose(
            position=self.position + np.asarray(delta, dtype=self.position.dtype),
            orientation=self.orientation,
            frame=self.frame,
        )


@dataclass(frozen=True)
class Vec3:
    """Tiny helper for places where a 3-vector field is clearer than a
    bare ndarray (e.g. config files)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float32)
