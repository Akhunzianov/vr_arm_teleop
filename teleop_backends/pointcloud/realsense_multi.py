"""Multi-camera RealSense fusion.

Connects to N RealSense cameras simultaneously and fuses their output
into a single world-frame point cloud. Per-camera extrinsics come from
a config file (a JSON/TOML of 4x4 transforms keyed by camera serial).

Each frame: spin all pipelines, deproject each camera's depth into its
own frame, transform to world via the extrinsic, concat. Optionally
voxel-downsample for stability + lower bandwidth.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from teleop_core.point_cloud import PointCloudFrame, PointCloudSource


@dataclass(frozen=True)
class CameraConfig:
    """One camera's identity + placement in the world."""
    serial: str
    extrinsic_world_from_cam: np.ndarray   # (4, 4) row-major
    width: int = 640
    height: int = 480
    fps: int = 30


class MultiRealSenseSource(PointCloudSource):
    """Fuses N RealSenses into one world-frame point cloud per ``grab()``."""

    def __init__(
        self,
        cameras: tuple[CameraConfig, ...],
        downsample: int = 4,
        z_min: float = 0.15,
        z_max: float = 2.5,
        workspace_crop: Optional[tuple[np.ndarray, np.ndarray]] = None,
        voxel_size: Optional[float] = None,
    ) -> None:
        self._cameras = cameras
        self._downsample = downsample
        self._z_min = z_min
        self._z_max = z_max
        self._workspace_crop = workspace_crop
        self._voxel_size = voxel_size

    @classmethod
    def from_config_file(cls, path: Path, **kwargs) -> "MultiRealSenseSource":
        """Build from a JSON/TOML file enumerating cameras + extrinsics."""
        raise NotImplementedError

    async def start(self) -> None:
        raise NotImplementedError

    async def stop(self) -> None:
        raise NotImplementedError

    async def grab(self) -> Optional[PointCloudFrame]:
        raise NotImplementedError
