"""Point-cloud source backed by pybullet's depth renderer.

Lets you drive the entire VR loop against a simulated scene -- useful
when the real cameras aren't set up yet, or for unit-style end-to-end
tests. Each frame, pose 1 or more virtual cameras inside a pybullet
client, ``getCameraImage`` for depth+color, deproject, fuse, return.

Behaviour mirrors :class:`MultiRealSenseSource`, just with sim cameras
instead of real ones.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_core.point_cloud import PointCloudFrame, PointCloudSource


@dataclass(frozen=True)
class VirtualCameraConfig:
    """One virtual camera placement + intrinsics for the sim renderer."""
    extrinsic_world_from_cam: np.ndarray   # (4, 4)
    width: int = 320
    height: int = 240
    fov_y_deg: float = 60.0
    near: float = 0.05
    far: float = 4.0


class PybulletPointCloudSource(PointCloudSource):
    """Renders synthetic depth from a running pybullet client."""

    def __init__(
        self,
        client_id: int,
        cameras: tuple[VirtualCameraConfig, ...],
    ) -> None:
        self._client_id = client_id
        self._cameras = cameras

    async def start(self) -> None:
        raise NotImplementedError

    async def stop(self) -> None:
        raise NotImplementedError

    async def grab(self) -> Optional[PointCloudFrame]:
        raise NotImplementedError
