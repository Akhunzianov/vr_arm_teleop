"""Two-camera ChArUco calibration helpers.

The calibration output uses OpenCV optical camera frames:
``+X`` right in the image, ``+Y`` down, and ``+Z`` forward through the lens.
The runtime config still stores transforms as ``world_from_camera`` matrices.
"""

from __future__ import annotations

import abc
import importlib
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class CameraDescriptor:
    """Camera metadata written beside the solved extrinsic."""

    name: str
    camera_type: str
    serial: str | None
    width: int
    height: int
    fps: int
    camera_matrix: list[list[float]]
    distortion: list[float]
    resolution: str | None = None
    depth_mode: str | None = None


@dataclass(frozen=True)
class CalibrationObservation:
    """One paired ChArUco observation from both cameras."""

    arm_camera_from_board: np.ndarray
    external_camera_from_board: np.ndarray
    arm_reprojection_error: float
    external_reprojection_error: float
    corner_count: int


@dataclass(frozen=True)
class CalibrationResult:
    """Solved extrinsics in OpenCV optical-frame convention."""

    world_from_arm_camera: np.ndarray
    world_from_external_camera: np.ndarray
    arm_camera_from_external_camera: np.ndarray
    external_camera_from_arm_camera: np.ndarray
    accepted_samples: int
    rejected_samples: int
    mean_arm_reprojection_error: float
    mean_external_reprojection_error: float
    translation_std_m: float
    rotation_std_deg: float


@dataclass(frozen=True)
class CalibrationDetection:
    """One board pose detected in one camera image."""

    camera_from_board: np.ndarray
    reprojection_error: float
    corner_count: int


@dataclass(frozen=True)
class CalibratedConfigWriteResult:
    """Metadata for an atomic hardware camera config update."""

    path: Path
    backup_path: Path | None
    updated_camera_names: tuple[str, ...]


@dataclass(frozen=True)
class _ExtrinsicSample:
    world_from_camera: np.ndarray
    reprojection_error: float
    corner_count: int
    timestamp: float


@dataclass(frozen=True)
class CharucoBoardSpec:
    """Metric ChArUco board description supplied from the CLI."""

    squares_x: int
    squares_y: int
    square_length: float
    marker_length: float
    dictionary: str

    def __post_init__(self) -> None:
        if self.squares_x < 2 or self.squares_y < 2:
            raise ValueError("ChArUco board requires at least 2 squares on each axis")
        if self.square_length <= 0.0:
            raise ValueError("square_length must be positive")
        if self.marker_length <= 0.0:
            raise ValueError("marker_length must be positive")
        if self.marker_length >= self.square_length:
            raise ValueError("marker_length must be smaller than square_length")
        if not self.dictionary:
            raise ValueError("dictionary must be non-empty")

    def create_cv2_board(self):
        cv2 = _require_cv2_aruco()
        dictionary = _cv2_aruco_dictionary(cv2, self.dictionary)
        if hasattr(cv2.aruco, "CharucoBoard"):
            return cv2.aruco.CharucoBoard(
                (self.squares_x, self.squares_y),
                self.square_length,
                self.marker_length,
                dictionary,
            )
        return cv2.aruco.CharucoBoard_create(
            self.squares_x,
            self.squares_y,
            self.square_length,
            self.marker_length,
            dictionary,
        )


def select_anchor_camera(config, *, anchor_name: str | None, camera_link: str):
    """Select the FK-trusted anchor camera from a hardware config."""

    cameras = tuple(getattr(config, "cameras", ()))
    if anchor_name:
        for camera in cameras:
            if camera.name == anchor_name:
                return camera
        raise ValueError(f"anchor camera {anchor_name!r} is not enabled in the config")

    matches = [
        camera for camera in cameras
        if getattr(camera, "urdf_link", None) == camera_link
    ]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        names = ", ".join(camera.name for camera in matches)
        raise ValueError(
            f"multiple cameras use URDF link {camera_link!r}; pass --anchor-camera "
            f"explicitly ({names})"
        )
    raise ValueError(
        f"no enabled camera has urdf_link {camera_link!r}; pass --anchor-camera"
    )


class AnchoredExtrinsicOptimizer:
    """Rolling camera-to-world optimizer using one FK-trusted anchor camera."""

    def __init__(
        self,
        *,
        anchor_name: str,
        target_names: Iterable[str],
        min_samples: int = 10,
        rolling_window: int = 40,
        min_corners: int = 12,
        max_reprojection_error_px: float = 2.0,
        max_pair_translation_deviation_m: float = 0.05,
        max_pair_rotation_deviation_deg: float = 5.0,
        translation_stability_m: float = 0.01,
        rotation_stability_deg: float = 1.0,
        stability_seconds: float = 2.0,
    ) -> None:
        if min_samples <= 0:
            raise ValueError("min_samples must be positive")
        if rolling_window < min_samples:
            raise ValueError("rolling_window must be >= min_samples")
        self.anchor_name = str(anchor_name)
        self.target_names = tuple(str(name) for name in target_names)
        if self.anchor_name in self.target_names:
            raise ValueError("anchor camera cannot also be a target")
        self._min_samples = int(min_samples)
        self._rolling_window = int(rolling_window)
        self._min_corners = int(min_corners)
        self._max_reprojection = float(max_reprojection_error_px)
        self._max_pair_translation = float(max_pair_translation_deviation_m)
        self._max_pair_rotation = math.radians(float(max_pair_rotation_deviation_deg))
        self._translation_stability = float(translation_stability_m)
        self._rotation_stability = float(rotation_stability_deg)
        self._stability_seconds = float(stability_seconds)
        self._samples: dict[str, list[_ExtrinsicSample]] = {
            name: [] for name in self.target_names
        }
        self._rejected: dict[str, int] = {name: 0 for name in self.target_names}
        self._latest_world_from_anchor: np.ndarray | None = None
        self._latest_timestamp: float | None = None

    def add_frame(
        self,
        *,
        world_from_anchor_camera: np.ndarray,
        detections: dict[str, CalibrationDetection],
        timestamp: float,
    ) -> None:
        """Consume one synchronized detection batch from all live cameras."""

        world_from_anchor_camera = _as_transform(world_from_anchor_camera)
        self._latest_world_from_anchor = world_from_anchor_camera
        self._latest_timestamp = float(timestamp)
        anchor = detections.get(self.anchor_name)
        if anchor is None or not self._valid_detection(anchor):
            for name in self.target_names:
                if name in detections:
                    self._rejected[name] += 1
            return

        anchor_from_board = _as_transform(anchor.camera_from_board)
        for name in self.target_names:
            detection = detections.get(name)
            if detection is None:
                continue
            if not self._valid_detection(detection):
                self._rejected[name] += 1
                continue
            world_from_camera = (
                world_from_anchor_camera
                @ anchor_from_board
                @ invert_transform(detection.camera_from_board)
            )
            sample = _ExtrinsicSample(
                world_from_camera=world_from_camera,
                reprojection_error=max(
                    float(anchor.reprojection_error),
                    float(detection.reprojection_error),
                ),
                corner_count=min(int(anchor.corner_count), int(detection.corner_count)),
                timestamp=float(timestamp),
            )
            samples = self._samples[name]
            samples.append(sample)
            if len(samples) > self._rolling_window:
                del samples[:-self._rolling_window]

    def status(self) -> dict:
        targets = {
            name: self._target_status(name)
            for name in self.target_names
        }
        return {
            "mode": "continuous_calibration",
            "anchor_camera": self.anchor_name,
            "all_stable": bool(targets) and all(
                target["stable"] for target in targets.values()
            ),
            "targets": targets,
        }

    def stable_transforms(self, *, include_anchor: bool = True) -> dict[str, np.ndarray]:
        transforms: dict[str, np.ndarray] = {}
        if include_anchor and self._latest_world_from_anchor is not None:
            transforms[self.anchor_name] = self._latest_world_from_anchor.copy()
        for name, status in self.status()["targets"].items():
            if status["stable"] and status["world_from_camera"] is not None:
                transforms[name] = np.asarray(
                    status["world_from_camera"],
                    dtype=np.float64,
                )
        return transforms

    def current_transforms(self, *, include_anchor: bool = True) -> dict[str, np.ndarray]:
        """Return the latest anchor plus provisional target solutions."""

        transforms: dict[str, np.ndarray] = {}
        if include_anchor and self._latest_world_from_anchor is not None:
            transforms[self.anchor_name] = self._latest_world_from_anchor.copy()
        for name, status in self.status()["targets"].items():
            if status["world_from_camera"] is not None:
                transforms[name] = np.asarray(
                    status["world_from_camera"],
                    dtype=np.float64,
                )
        return transforms

    def _valid_detection(self, detection: CalibrationDetection) -> bool:
        return (
            int(detection.corner_count) >= self._min_corners
            and float(detection.reprojection_error) <= self._max_reprojection
        )

    def _target_status(self, name: str) -> dict:
        samples = self._samples[name]
        base = {
            "stable": False,
            "accepted_samples": 0,
            "rejected_samples": int(self._rejected[name]),
            "candidate_samples": len(samples),
            "reprojection_error_px": None,
            "translation_std_m": None,
            "rotation_std_deg": None,
            "world_from_camera": None,
            "last_timestamp": None if not samples else float(samples[-1].timestamp),
        }
        if len(samples) < self._min_samples:
            return base

        preliminary = _average_transforms(sample.world_from_camera for sample in samples)
        accepted: list[_ExtrinsicSample] = []
        translation_errors = []
        rotation_errors = []
        for sample in samples:
            transform = _as_transform(sample.world_from_camera)
            translation_error = float(np.linalg.norm(transform[:3, 3] - preliminary[:3, 3]))
            rotation_error = _rotation_angle(preliminary[:3, :3].T @ transform[:3, :3])
            if (
                translation_error <= self._max_pair_translation
                and rotation_error <= self._max_pair_rotation
            ):
                accepted.append(sample)
                translation_errors.append(translation_error)
                rotation_errors.append(rotation_error)

        if len(accepted) < self._min_samples:
            rejected = self._rejected[name] + (len(samples) - len(accepted))
            return {**base, "rejected_samples": int(rejected)}

        solution = _average_transforms(sample.world_from_camera for sample in accepted)
        reprojection = float(np.mean([sample.reprojection_error for sample in accepted]))
        translation_std = float(np.std(translation_errors)) if translation_errors else 0.0
        rotation_std = (
            float(np.degrees(np.std(rotation_errors)))
            if rotation_errors else 0.0
        )
        time_span = float(accepted[-1].timestamp - accepted[0].timestamp)
        stable = (
            translation_std <= self._translation_stability
            and rotation_std <= self._rotation_stability
            and time_span >= self._stability_seconds
        )
        return {
            **base,
            "stable": bool(stable),
            "accepted_samples": len(accepted),
            "rejected_samples": int(self._rejected[name] + len(samples) - len(accepted)),
            "reprojection_error_px": reprojection,
            "translation_std_m": translation_std,
            "rotation_std_deg": rotation_std,
            "world_from_camera": solution.tolist(),
        }


class CameraFeed(abc.ABC):
    """Live color camera interface used by the interactive script."""

    @abc.abstractmethod
    def start(self) -> None:
        """Open the camera."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Release camera resources."""

    @abc.abstractmethod
    def read_color(self) -> Optional[np.ndarray]:
        """Return an RGB image, or ``None`` if a frame is unavailable."""

    @abc.abstractmethod
    def descriptor(self) -> CameraDescriptor:
        """Return camera metadata and intrinsics."""


class RealSenseColorFeed(CameraFeed):
    """RGB feed backed by ``pyrealsense2`` with lazy SDK import."""

    def __init__(
        self,
        *,
        serial: str | None,
        width: int,
        height: int,
        fps: int,
        name: str = "realsense",
    ) -> None:
        self._serial = serial
        self._width = width
        self._height = height
        self._fps = fps
        self._name = name
        self._rs = None
        self._pipeline = None
        self._profile = None

    def start(self) -> None:
        try:
            rs = importlib.import_module("pyrealsense2")
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "RealSense color feed requires pyrealsense2. Install Intel "
                "RealSense SDK Python bindings on the capture host."
            ) from exc
        pipeline = rs.pipeline()
        config = rs.config()
        if self._serial:
            config.enable_device(self._serial)
        config.enable_stream(
            rs.stream.color,
            self._width,
            self._height,
            rs.format.rgb8,
            self._fps,
        )
        self._profile = pipeline.start(config)
        self._rs = rs
        self._pipeline = pipeline

    def stop(self) -> None:
        pipeline = self._pipeline
        self._pipeline = None
        if pipeline is not None:
            pipeline.stop()

    def read_color(self) -> Optional[np.ndarray]:
        if self._pipeline is None:
            return None
        frames = self._pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data()).copy()

    def descriptor(self) -> CameraDescriptor:
        intr = self._intrinsics()
        return CameraDescriptor(
            name=self._name,
            camera_type="realsense",
            serial=self._serial,
            width=self._width,
            height=self._height,
            fps=self._fps,
            camera_matrix=[
                [float(intr.fx), 0.0, float(intr.ppx)],
                [0.0, float(intr.fy), float(intr.ppy)],
                [0.0, 0.0, 1.0],
            ],
            distortion=[float(v) for v in getattr(intr, "coeffs", [])],
        )

    def _intrinsics(self):
        if self._profile is None or self._rs is None:
            raise RuntimeError("RealSense feed must be started before reading intrinsics")
        stream = self._profile.get_stream(self._rs.stream.color)
        return stream.as_video_stream_profile().get_intrinsics()


class ZedColorFeed(CameraFeed):
    """Left RGB feed backed by ``pyzed.sl`` with lazy SDK import."""

    def __init__(
        self,
        *,
        serial: str | None,
        resolution: str,
        fps: int,
        name: str = "zed",
    ) -> None:
        self._serial = serial
        self._resolution = resolution
        self._fps = fps
        self._name = name
        self._sl = None
        self._zed = None
        self._mat = None

    def start(self) -> None:
        try:
            sl = importlib.import_module("pyzed.sl")
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "ZED color feed requires pyzed.sl. Install the Stereolabs "
                "ZED SDK Python API on the capture host."
            ) from exc
        init = sl.InitParameters()
        if self._serial:
            init.set_from_serial_number(int(self._serial))
        init.camera_resolution = _zed_enum(sl.RESOLUTION, self._resolution, "HD720")
        init.camera_fps = self._fps
        init.coordinate_units = sl.UNIT.METER

        zed = sl.Camera()
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"failed to open ZED camera {self._name!r}: {status}")
        self._sl = sl
        self._zed = zed
        self._mat = sl.Mat()

    def stop(self) -> None:
        zed = self._zed
        self._zed = None
        if zed is not None:
            zed.close()

    def read_color(self) -> Optional[np.ndarray]:
        if self._zed is None or self._sl is None or self._mat is None:
            return None
        status = self._zed.grab(self._sl.RuntimeParameters())
        if status != self._sl.ERROR_CODE.SUCCESS:
            return None
        self._zed.retrieve_image(self._mat, self._sl.VIEW.LEFT)
        image = np.asarray(self._mat.get_data())
        if image.ndim != 3 or image.shape[2] < 3:
            return None
        # ZED image data is BGRA/BGR-like; return RGB for a consistent feed API.
        return image[:, :, 2::-1].copy()

    def descriptor(self) -> CameraDescriptor:
        if self._zed is None:
            raise RuntimeError("ZED feed must be started before reading intrinsics")
        info = self._zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters.left_cam
        width = int(getattr(calib, "image_size", (0, 0))[0] or 0)
        height = int(getattr(calib, "image_size", (0, 0))[1] or 0)
        return CameraDescriptor(
            name=self._name,
            camera_type="zed2i",
            serial=self._serial,
            width=width,
            height=height,
            fps=self._fps,
            camera_matrix=[
                [float(calib.fx), 0.0, float(calib.cx)],
                [0.0, float(calib.fy), float(calib.cy)],
                [0.0, 0.0, 1.0],
            ],
            distortion=[float(v) for v in getattr(calib, "disto", [])],
            resolution=self._resolution,
        )


class JointStateProvider(abc.ABC):
    """Boundary for the future live 7-DoF arm SDK adapter."""

    @abc.abstractmethod
    def read_joint_state(self) -> dict[str, float]:
        """Return joint positions keyed by URDF joint name."""


class UnavailableJointStateProvider(JointStateProvider):
    """Placeholder that fails loudly until the real arm SDK is wired in."""

    def __init__(self, message: str | None = None) -> None:
        self._message = message or (
            "live robot joint-state provider is not implemented yet; "
            "wire the 7-DoF arm SDK adapter here"
        )

    def read_joint_state(self) -> dict[str, float]:
        raise RuntimeError(self._message)


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


class UrdfKinematicTree:
    """Minimal FK for a URDF chain from base link to camera link."""

    def __init__(self, joints_by_child: dict[str, _UrdfJoint]) -> None:
        self._joints_by_child = joints_by_child

    @classmethod
    def from_file(cls, path: Path) -> "UrdfKinematicTree":
        root = ET.parse(path).getroot()
        joints: dict[str, _UrdfJoint] = {}
        for elem in root.findall("joint"):
            name = elem.attrib["name"]
            joint_type = elem.attrib.get("type", "fixed")
            parent = elem.find("parent").attrib["link"]
            child = elem.find("child").attrib["link"]
            origin_elem = elem.find("origin")
            xyz = _parse_floats(origin_elem.attrib.get("xyz", "0 0 0") if origin_elem is not None else "0 0 0")
            rpy = _parse_floats(origin_elem.attrib.get("rpy", "0 0 0") if origin_elem is not None else "0 0 0")
            axis_elem = elem.find("axis")
            axis = _parse_floats(axis_elem.attrib.get("xyz", "1 0 0") if axis_elem is not None else "1 0 0")
            joints[child] = _UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin=transform_from_rt(_rpy_matrix(rpy), xyz),
                axis=_normalize(axis),
            )
        return cls(joints)

    def transform(
        self,
        base_link: str,
        target_link: str,
        joint_positions: dict[str, float],
    ) -> np.ndarray:
        chain: list[_UrdfJoint] = []
        link = target_link
        while link != base_link:
            joint = self._joints_by_child.get(link)
            if joint is None:
                raise ValueError(f"no URDF chain from {base_link!r} to {target_link!r}")
            chain.append(joint)
            link = joint.parent
        transform = np.eye(4, dtype=np.float64)
        for joint in reversed(chain):
            transform = transform @ joint.origin @ _joint_motion(joint, joint_positions)
        return transform


def transform_from_rt(rotation: Iterable[Iterable[float]], translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = _as_transform(transform)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def rotation_matrix_from_axis_angle(axis: Iterable[float], angle: float) -> np.ndarray:
    axis = _normalize(np.asarray(axis, dtype=np.float64))
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def estimate_two_camera_extrinsics(
    observations: Iterable[CalibrationObservation],
    *,
    world_from_arm_camera: np.ndarray,
    min_samples: int = 10,
    min_corners: int = 12,
    max_reprojection_error_px: float = 2.0,
    max_pair_translation_deviation_m: float = 0.05,
    max_pair_rotation_deviation_deg: float = 5.0,
) -> CalibrationResult:
    """Solve camera extrinsics from paired board poses.

    Each OpenCV ChArUco pose is ``camera_from_board``. A paired sample gives
    ``arm_camera_from_external_camera = arm_camera_from_board @
    inverse(external_camera_from_board)``.
    """

    world_from_arm_camera = _as_transform(world_from_arm_camera)
    candidate_pairs = []
    rejected = 0
    arm_errors = []
    external_errors = []
    for observation in observations:
        if (
            observation.corner_count < min_corners
            or observation.arm_reprojection_error > max_reprojection_error_px
            or observation.external_reprojection_error > max_reprojection_error_px
        ):
            rejected += 1
            continue
        arm_camera_from_board = _as_transform(observation.arm_camera_from_board)
        external_camera_from_board = _as_transform(observation.external_camera_from_board)
        candidate_pairs.append(
            arm_camera_from_board @ invert_transform(external_camera_from_board)
        )
        arm_errors.append(float(observation.arm_reprojection_error))
        external_errors.append(float(observation.external_reprojection_error))

    if len(candidate_pairs) < min_samples:
        raise ValueError(
            f"need at least {min_samples} valid paired samples, got {len(candidate_pairs)}"
        )

    preliminary = _average_transforms(candidate_pairs)
    accepted_pairs = []
    accepted_arm_errors = []
    accepted_external_errors = []
    translation_errors = []
    rotation_errors = []
    max_rotation_rad = math.radians(max_pair_rotation_deviation_deg)
    for pair, arm_error, external_error in zip(candidate_pairs, arm_errors, external_errors):
        translation_error = float(np.linalg.norm(pair[:3, 3] - preliminary[:3, 3]))
        rotation_error = _rotation_angle(preliminary[:3, :3].T @ pair[:3, :3])
        if (
            translation_error <= max_pair_translation_deviation_m
            and rotation_error <= max_rotation_rad
        ):
            accepted_pairs.append(pair)
            accepted_arm_errors.append(arm_error)
            accepted_external_errors.append(external_error)
            translation_errors.append(translation_error)
            rotation_errors.append(rotation_error)
        else:
            rejected += 1

    if len(accepted_pairs) < min_samples:
        raise ValueError(
            f"need at least {min_samples} inlier samples, got {len(accepted_pairs)}"
        )

    arm_camera_from_external_camera = _average_transforms(accepted_pairs)
    external_camera_from_arm_camera = invert_transform(arm_camera_from_external_camera)
    world_from_external_camera = world_from_arm_camera @ arm_camera_from_external_camera
    return CalibrationResult(
        world_from_arm_camera=world_from_arm_camera,
        world_from_external_camera=world_from_external_camera,
        arm_camera_from_external_camera=arm_camera_from_external_camera,
        external_camera_from_arm_camera=external_camera_from_arm_camera,
        accepted_samples=len(accepted_pairs),
        rejected_samples=rejected,
        mean_arm_reprojection_error=float(np.mean(accepted_arm_errors)),
        mean_external_reprojection_error=float(np.mean(accepted_external_errors)),
        translation_std_m=float(np.std(translation_errors)) if translation_errors else 0.0,
        rotation_std_deg=float(np.degrees(np.std(rotation_errors))) if rotation_errors else 0.0,
    )


def build_hardware_camera_config(
    result: CalibrationResult,
    *,
    arm_camera: CameraDescriptor,
    external_camera: CameraDescriptor,
) -> dict:
    """Build a JSON-serializable config compatible with hardware point clouds."""

    return {
        "cameras": [
            _camera_config_entry(arm_camera, result.world_from_arm_camera),
            _camera_config_entry(external_camera, result.world_from_external_camera),
        ]
    }


def _camera_config_entry(camera: CameraDescriptor, world_from_camera: np.ndarray) -> dict:
    world_from_camera = _as_transform(world_from_camera).tolist()
    entry = {
        "name": camera.name,
        "type": camera.camera_type,
        "serial": camera.serial,
        "width": camera.width,
        "height": camera.height,
        "fps": camera.fps,
        "calibrated": True,
        "world_from_camera": world_from_camera,
        "extrinsic_world_from_cam": world_from_camera,
        "intrinsics": {
            "camera_matrix": camera.camera_matrix,
            "distortion": camera.distortion,
        },
    }
    if camera.resolution is not None:
        entry["resolution"] = camera.resolution
    if camera.depth_mode is not None:
        entry["depth_mode"] = camera.depth_mode
    return entry


def write_calibrated_hardware_config(
    path: Path,
    world_from_camera_by_name: dict[str, np.ndarray],
    *,
    backup: bool = True,
) -> CalibratedConfigWriteResult:
    """Atomically update enabled camera extrinsics in an existing config."""

    path = Path(path)
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get("cameras"), list):
        raise ValueError("hardware camera config requires a top-level cameras list")

    backup_path = None
    if backup:
        backup_path = path.with_suffix(path.suffix + ".bak")
        if not backup_path.exists():
            shutil.copy2(path, backup_path)

    updated: list[str] = []
    for camera in data["cameras"]:
        if not isinstance(camera, dict):
            continue
        if camera.get("enabled", True) is False:
            continue
        name = str(camera.get("name", ""))
        if name not in world_from_camera_by_name:
            continue
        world_from_camera = _as_transform(world_from_camera_by_name[name]).tolist()
        camera["calibrated"] = True
        camera["world_from_camera"] = world_from_camera
        camera["extrinsic_world_from_cam"] = world_from_camera
        updated.append(name)

    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(tmp, path)
    return CalibratedConfigWriteResult(
        path=path,
        backup_path=backup_path,
        updated_camera_names=tuple(updated),
    )


def _average_transforms(transforms: Iterable[np.ndarray]) -> np.ndarray:
    transforms = [_as_transform(transform) for transform in transforms]
    translations = np.asarray([transform[:3, 3] for transform in transforms], dtype=np.float64)
    quaternions = np.asarray(
        [_quaternion_from_matrix(transform[:3, :3]) for transform in transforms],
        dtype=np.float64,
    )
    reference = quaternions[0]
    for i, quat in enumerate(quaternions):
        if np.dot(reference, quat) < 0.0:
            quaternions[i] = -quat
    accumulator = quaternions.T @ quaternions
    _, vectors = np.linalg.eigh(accumulator)
    quat = vectors[:, -1]
    if quat[3] < 0.0:
        quat = -quat
    return transform_from_rt(_matrix_from_quaternion(quat), np.median(translations, axis=0))


def _as_transform(value: np.ndarray) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    if not np.all(np.isfinite(transform)):
        raise ValueError("transform contains non-finite values")
    return transform


def _quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s,
                0.25 * s,
            ],
            dtype=np.float64,
        )
    diag = np.diag(rotation)
    idx = int(np.argmax(diag))
    if idx == 0:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        quat = [
            0.25 * s,
            (rotation[0, 1] + rotation[1, 0]) / s,
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[2, 1] - rotation[1, 2]) / s,
        ]
    elif idx == 1:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        quat = [
            (rotation[0, 1] + rotation[1, 0]) / s,
            0.25 * s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
        ]
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        quat = [
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            0.25 * s,
            (rotation[1, 0] - rotation[0, 1]) / s,
        ]
    return _normalize(np.asarray(quat, dtype=np.float64))


def _matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize(np.asarray(quaternion, dtype=np.float64))
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _rotation_angle(rotation: np.ndarray) -> float:
    cos_angle = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, cos_angle)))


def _joint_motion(joint: _UrdfJoint, positions: dict[str, float]) -> np.ndarray:
    if joint.joint_type in {"fixed"}:
        return np.eye(4, dtype=np.float64)
    value = float(positions.get(joint.name, 0.0))
    if joint.joint_type in {"revolute", "continuous"}:
        return transform_from_rt(rotation_matrix_from_axis_angle(joint.axis, value), [0, 0, 0])
    if joint.joint_type == "prismatic":
        return transform_from_rt(np.eye(3), joint.axis * value)
    raise ValueError(f"unsupported URDF joint type {joint.joint_type!r}")


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    rx = rotation_matrix_from_axis_angle([1, 0, 0], roll)
    ry = rotation_matrix_from_axis_angle([0, 1, 0], pitch)
    rz = rotation_matrix_from_axis_angle([0, 0, 1], yaw)
    return rz @ ry @ rx


def _parse_floats(value: str) -> np.ndarray:
    return np.asarray([float(part) for part in value.split()], dtype=np.float64)


def _normalize(vector: Iterable[float]) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        raise ValueError("cannot normalize zero-length vector")
    return vector / norm


def _require_cv2_aruco():
    try:
        cv2 = importlib.import_module("cv2")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "ChArUco calibration requires opencv-contrib-python with cv2.aruco"
        ) from exc
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("installed cv2 lacks aruco; install opencv-contrib-python")
    return cv2


def _cv2_aruco_dictionary(cv2, name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary {name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _zed_enum(enum_cls, value: str | None, default: str):
    name = (value or default).upper()
    try:
        return getattr(enum_cls, name)
    except AttributeError as exc:
        raise ValueError(f"unknown ZED enum value {name!r}") from exc


__all__ = [
    "AnchoredExtrinsicOptimizer",
    "CameraDescriptor",
    "CameraFeed",
    "CalibratedConfigWriteResult",
    "CalibrationDetection",
    "CalibrationObservation",
    "CalibrationResult",
    "CharucoBoardSpec",
    "JointStateProvider",
    "RealSenseColorFeed",
    "UnavailableJointStateProvider",
    "UrdfKinematicTree",
    "ZedColorFeed",
    "build_hardware_camera_config",
    "estimate_two_camera_extrinsics",
    "invert_transform",
    "rotation_matrix_from_axis_angle",
    "select_anchor_camera",
    "transform_from_rt",
    "write_calibrated_hardware_config",
]
