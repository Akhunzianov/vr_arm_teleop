"""Point-cloud source interface.

Any class that can produce a fused, world-frame point cloud implements
this. Concrete implementations live in :mod:`teleop_backends.pointcloud`.

Wire format (binary WebSocket frame; mirrors what the browser parses):

    [uint32 N][uint32 reserved=0]
    [N * int16 x_mm][N * int16 y_mm][N * int16 z_mm]   # world-frame, mm
    [N * uint8 r][N * uint8 g][N * uint8 b]

The serialization helper :func:`encode_frame` lives here so the same
binary layout is used by every backend without duplication.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PointCloudFrame:
    """A timestamped colored point cloud in the **world** frame.

    Concrete sources are responsible for transforming each camera's
    output into ``world`` before yielding this. Server-side code never
    needs to know how many cameras were fused.
    """

    points: np.ndarray   # shape (N, 3), float32, meters, world frame
    colors: np.ndarray   # shape (N, 3), uint8, RGB
    timestamp: float     # time.monotonic() seconds when captured

    @property
    def n_points(self) -> int:
        return int(self.points.shape[0])


class PointCloudSource(abc.ABC):
    """Anything that can produce :class:`PointCloudFrame` snapshots.

    Implementations: multi-RealSense fusion, single RealSense, replay of
    recorded clouds, synthetic generator, pybullet depth render. The
    server uses only this interface, so swapping is trivial.
    """

    @abc.abstractmethod
    async def start(self) -> None:
        """Open hardware / start capture threads. Idempotent."""

    @abc.abstractmethod
    async def stop(self) -> None:
        """Release resources. Must be safe to call after a failed start()."""

    @abc.abstractmethod
    async def grab(self) -> Optional[PointCloudFrame]:
        """Return the latest fused frame, or ``None`` if not ready yet.

        Should not block longer than ~1 frame period. The server polls
        this from its own loop and applies its own rate limit.
        """


def encode_frame(frame: PointCloudFrame) -> bytes:
    """Pack a :class:`PointCloudFrame` into the binary wire format.

    Positions are quantized to int16 millimeters; colors stay uint8. The
    JS client deserialises with typed-array views over the ArrayBuffer.
    """
    n = frame.n_points
    header = np.array([n, 0], dtype=np.uint32).tobytes()
    if n == 0:
        return header
    # Transpose to (3, N) and let astype produce a fresh C-contiguous
    # array, so tobytes() yields x[N] then y[N] then z[N] as the wire
    # format requires.
    pts_mm = np.rint(frame.points.T * 1000.0).astype(np.int16)
    cols = frame.colors.T.astype(np.uint8)
    return header + pts_mm.tobytes() + cols.tobytes()
