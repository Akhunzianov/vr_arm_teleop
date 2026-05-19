"""Procedural point cloud source -- for development without a camera.

Generates a recognisable, animated cloud (a wavy plane) so you can verify
the streaming pipeline end-to-end. Useful when the RealSense isn't
plugged in or for CI.

Geometry is emitted directly in the operator's local-floor space (until
proper world-frame calibration exists, see README "Coordinate frames"):
the plane sits ~60 cm in front of the headset, around eye height, so it
shows up as soon as the user enters VR.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from teleop_core.point_cloud import PointCloudFrame, PointCloudSource


class MockPointCloudSource(PointCloudSource):
    """Animated synthetic cloud. ~5000 points, sub-millisecond per frame."""

    def __init__(self, point_count: int = 5000) -> None:
        side = max(2, int(np.sqrt(point_count)))
        xs = np.linspace(-0.5, 0.5, side, dtype=np.float32)
        zs = np.linspace(-0.5, 0.5, side, dtype=np.float32)
        xx, zz = np.meshgrid(xs, zs)
        self._x = xx.reshape(-1)
        self._z = zz.reshape(-1)
        self._started = False
        self._t0 = time.monotonic()

    async def start(self) -> None:
        self._t0 = time.monotonic()
        self._started = True

    async def stop(self) -> None:
        self._started = False

    async def grab(self) -> Optional[PointCloudFrame]:
        if not self._started:
            return None
        t = time.monotonic() - self._t0
        y = (1.0
             + 0.1 * np.sin(4.0 * self._x + t).astype(np.float32)
             + 0.1 * np.cos(4.0 * self._z + t).astype(np.float32))
        # Push the plane ~60 cm in front of the boot location.
        points = np.stack([self._x, y, self._z - 0.6], axis=1).astype(np.float32)
        # Heat-ish coloring by height so the wave is obvious.
        norm = np.clip((y - 0.8) / 0.4, 0.0, 1.0)
        r = (norm * 255).astype(np.uint8)
        g = (np.abs(norm - 0.5) * 2.0 * 255).astype(np.uint8)
        b = ((1.0 - norm) * 255).astype(np.uint8)
        colors = np.stack([r, g, b], axis=1)
        return PointCloudFrame(points=points, colors=colors, timestamp=time.monotonic())
