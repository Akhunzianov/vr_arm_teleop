"""Calibrate external + arm-mounted cameras with a ChArUco board.

The solver trusts the URDF/FK pose of the arm-mounted camera in the robot
base frame, then estimates the external camera from paired ChArUco board
poses. Output matrices use OpenCV optical camera frames.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from teleop_backends.camera_calibration import (
    CameraDescriptor,
    CalibrationObservation,
    CameraFeed,
    CharucoBoardSpec,
    JointStateProvider,
    RealSenseColorFeed,
    UnavailableJointStateProvider,
    UrdfKinematicTree,
    ZedColorFeed,
    build_hardware_camera_config,
    estimate_two_camera_extrinsics,
    transform_from_rt,
)


@dataclass(frozen=True)
class _Detection:
    camera_from_board: np.ndarray
    reprojection_error: float
    corner_count: int
    overlay_rgb: np.ndarray


class JsonJointStateProvider(JointStateProvider):
    """Development provider for a fixed calibration pose snapshot."""

    def __init__(self, path: Path) -> None:
        self._path = path

    def read_joint_state(self) -> dict[str, float]:
        data = json.loads(self._path.read_text())
        if not isinstance(data, dict):
            raise ValueError("joint state JSON must be an object keyed by URDF joint name")
        return {str(name): float(value) for name, value in data.items()}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)

    ap.add_argument("--arm-camera", choices=("realsense", "zed"), required=True)
    ap.add_argument("--arm-name", default="arm-camera")
    ap.add_argument("--arm-serial", default=None)
    ap.add_argument("--external-camera", choices=("realsense", "zed"), required=True)
    ap.add_argument("--external-name", default="external-camera")
    ap.add_argument("--external-serial", default=None)

    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--zed-resolution", default="HD720")

    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--base-link", required=True)
    ap.add_argument("--camera-link", required=True)
    ap.add_argument(
        "--joint-state-json",
        type=Path,
        default=None,
        help="temporary fixed-pose joint snapshot; omit when the live SDK provider is wired",
    )

    ap.add_argument("--squares-x", type=int, required=True)
    ap.add_argument("--squares-y", type=int, required=True)
    ap.add_argument("--square-length", type=float, required=True)
    ap.add_argument("--marker-length", type=float, required=True)
    ap.add_argument("--dictionary", default="DICT_5X5_100")

    ap.add_argument("--min-samples", type=int, default=10)
    ap.add_argument("--min-corners", type=int, default=12)
    ap.add_argument("--max-reprojection-error", type=float, default=2.0)
    ap.add_argument("--output", type=Path, default=Path("config/cameras.json"))
    return ap.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    run_interactive_calibration(args)


def run_interactive_calibration(args: argparse.Namespace) -> None:
    cv2 = _require_cv2_aruco()
    board_spec = CharucoBoardSpec(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length=args.square_length,
        marker_length=args.marker_length,
        dictionary=args.dictionary,
    )
    board = board_spec.create_cv2_board()

    joint_provider = _make_joint_state_provider(args)
    joint_state = joint_provider.read_joint_state()
    world_from_arm_camera = UrdfKinematicTree.from_file(args.urdf).transform(
        args.base_link,
        args.camera_link,
        joint_state,
    )

    arm_feed = _make_feed(
        args.arm_camera,
        name=args.arm_name,
        serial=args.arm_serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        zed_resolution=args.zed_resolution,
    )
    external_feed = _make_feed(
        args.external_camera,
        name=args.external_name,
        serial=args.external_serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        zed_resolution=args.zed_resolution,
    )

    observations: list[CalibrationObservation] = []
    try:
        arm_feed.start()
        external_feed.start()
        arm_descriptor = arm_feed.descriptor()
        external_descriptor = external_feed.descriptor()
        print("[calib] Press SPACE/C to accept a stable paired sample; Q/ESC quits.")
        while True:
            arm_image = arm_feed.read_color()
            external_image = external_feed.read_color()
            if arm_image is None or external_image is None:
                continue

            arm_detection = _detect_board_pose(
                cv2,
                board,
                arm_image,
                arm_descriptor,
                min_corners=args.min_corners,
            )
            external_detection = _detect_board_pose(
                cv2,
                board,
                external_image,
                external_descriptor,
                min_corners=args.min_corners,
            )

            display = _display_pair(
                cv2,
                arm_detection.overlay_rgb if arm_detection else arm_image,
                external_detection.overlay_rgb if external_detection else external_image,
                len(observations),
            )
            cv2.imshow("two-camera ChArUco calibration", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                if len(observations) >= args.min_samples:
                    break
                raise SystemExit("calibration cancelled before enough samples were captured")
            if key in (ord(" "), ord("c")):
                if arm_detection is None or external_detection is None:
                    print("[calib] rejected: both cameras must detect enough ChArUco corners")
                    continue
                if (
                    arm_detection.reprojection_error > args.max_reprojection_error
                    or external_detection.reprojection_error > args.max_reprojection_error
                ):
                    print(
                        "[calib] rejected: reprojection error too high "
                        f"(arm={arm_detection.reprojection_error:.3f}px, "
                        f"external={external_detection.reprojection_error:.3f}px)"
                    )
                    continue
                observations.append(
                    CalibrationObservation(
                        arm_camera_from_board=arm_detection.camera_from_board,
                        external_camera_from_board=external_detection.camera_from_board,
                        arm_reprojection_error=arm_detection.reprojection_error,
                        external_reprojection_error=external_detection.reprojection_error,
                        corner_count=min(arm_detection.corner_count, external_detection.corner_count),
                    )
                )
                print(
                    f"[calib] accepted sample {len(observations)} "
                    f"(arm={arm_detection.reprojection_error:.3f}px, "
                    f"external={external_detection.reprojection_error:.3f}px)"
                )
                if len(observations) >= args.min_samples:
                    print("[calib] minimum sample count reached; press Q to stop or capture more.")
    finally:
        arm_feed.stop()
        external_feed.stop()
        try:
            cv2.destroyWindow("two-camera ChArUco calibration")
        except Exception:
            pass

    result = estimate_two_camera_extrinsics(
        observations,
        world_from_arm_camera=world_from_arm_camera,
        min_samples=args.min_samples,
        min_corners=args.min_corners,
        max_reprojection_error_px=args.max_reprojection_error,
    )
    output = build_hardware_camera_config(
        result,
        arm_camera=arm_descriptor,
        external_camera=external_descriptor,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2))
    print(f"[calib] wrote {args.output}")
    print(
        "[calib] metrics: "
        f"accepted={result.accepted_samples}, rejected={result.rejected_samples}, "
        f"arm_reproj={result.mean_arm_reprojection_error:.3f}px, "
        f"external_reproj={result.mean_external_reprojection_error:.3f}px, "
        f"translation_std={result.translation_std_m:.4f}m, "
        f"rotation_std={result.rotation_std_deg:.3f}deg"
    )


def _make_joint_state_provider(args: argparse.Namespace) -> JointStateProvider:
    if args.joint_state_json is not None:
        return JsonJointStateProvider(args.joint_state_json)
    return UnavailableJointStateProvider(
        "live robot joint-state SDK adapter is not implemented yet; "
        "provide --joint-state-json for a fixed-pose snapshot or wire the SDK provider"
    )


def _make_feed(
    kind: str,
    *,
    name: str,
    serial: str | None,
    width: int,
    height: int,
    fps: int,
    zed_resolution: str,
) -> CameraFeed:
    if kind == "realsense":
        return RealSenseColorFeed(
            serial=serial,
            width=width,
            height=height,
            fps=fps,
            name=name,
        )
    if kind == "zed":
        return ZedColorFeed(
            serial=serial,
            resolution=zed_resolution,
            fps=fps,
            name=name,
        )
    raise ValueError(f"unsupported camera type {kind!r}")


def _detect_board_pose(
    cv2,
    board,
    image_rgb: np.ndarray,
    descriptor: CameraDescriptor,
    *,
    min_corners: int,
) -> Optional[_Detection]:
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    camera_matrix = np.asarray(descriptor.camera_matrix, dtype=np.float64)
    distortion = np.asarray(descriptor.distortion, dtype=np.float64)
    dictionary = board.getDictionary() if hasattr(board, "getDictionary") else board.dictionary
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    overlay = image_rgb.copy()
    if marker_ids is None or len(marker_ids) == 0:
        return None

    cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        cameraMatrix=camera_matrix,
        distCoeffs=distortion,
    )
    if charuco_ids is None or charuco_corners is None or len(charuco_ids) < min_corners:
        return None

    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners,
        charuco_ids,
        board,
        camera_matrix,
        distortion,
        None,
        None,
    )
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    camera_from_board = transform_from_rt(rotation, np.asarray(tvec, dtype=np.float64).reshape(3))
    reprojection_error = _charuco_reprojection_error(
        cv2,
        board,
        charuco_corners,
        charuco_ids,
        rvec,
        tvec,
        camera_matrix,
        distortion,
    )
    cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)
    cv2.drawFrameAxes(overlay, camera_matrix, distortion, rvec, tvec, 0.05)
    return _Detection(
        camera_from_board=camera_from_board,
        reprojection_error=reprojection_error,
        corner_count=int(len(charuco_ids)),
        overlay_rgb=overlay,
    )


def _charuco_reprojection_error(
    cv2,
    board,
    charuco_corners,
    charuco_ids,
    rvec,
    tvec,
    camera_matrix,
    distortion,
) -> float:
    if hasattr(board, "getChessboardCorners"):
        chessboard_corners = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    else:
        chessboard_corners = np.asarray(board.chessboardCorners, dtype=np.float32)
    object_points = chessboard_corners[np.asarray(charuco_ids, dtype=np.int32).reshape(-1)]
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    projected = projected.reshape(-1, 2)
    observed = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((projected - observed) ** 2, axis=1))))


def _display_pair(cv2, arm_rgb: np.ndarray, external_rgb: np.ndarray, sample_count: int) -> np.ndarray:
    target_height = min(arm_rgb.shape[0], external_rgb.shape[0], 720)
    arm = _resize_to_height(cv2, arm_rgb, target_height)
    external = _resize_to_height(cv2, external_rgb, target_height)
    display_rgb = np.concatenate([arm, external], axis=1)
    cv2.putText(
        display_rgb,
        f"samples: {sample_count}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return cv2.cvtColor(display_rgb, cv2.COLOR_RGB2BGR)


def _resize_to_height(cv2, image: np.ndarray, target_height: int) -> np.ndarray:
    if image.shape[0] == target_height:
        return image
    scale = target_height / image.shape[0]
    width = max(1, int(round(image.shape[1] * scale)))
    return cv2.resize(image, (width, target_height), interpolation=cv2.INTER_AREA)


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


if __name__ == "__main__":
    main()
