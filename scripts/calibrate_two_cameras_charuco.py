"""Continuously calibrate configured cameras with a ChArUco board.

The live solver trusts the URDF/FK pose of the arm-mounted anchor camera
in the robot base frame. Every other enabled camera is optimized from
frames where both it and the anchor see the same ChArUco board. Output
matrices use OpenCV optical camera frames and are written as
``world_from_camera`` matrices compatible with the hardware point-cloud
backend.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import importlib
import json
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional
from urllib.parse import quote

from aiohttp import WSMsgType, web
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from teleop_backends.camera_calibration import (
    AnchoredExtrinsicOptimizer,
    CameraDescriptor,
    CalibrationDetection,
    CharucoBoardSpec,
    JointStateProvider,
    UrdfKinematicTree,
    select_anchor_camera,
    transform_from_rt,
    write_calibrated_hardware_config,
)
from teleop_backends.pointcloud.hardware import (
    HardwareCameraConfig,
    HardwarePointCloudConfig,
    CameraPointCloudReader,
    default_reader_factory,
    fuse_camera_frames,
    load_hardware_config,
)
from teleop_backends.robot.rc5_state import (
    RC5_ARM_JOINT_NAMES,
    RC5JointStateReader,
)
from teleop_core.point_cloud import PointCloudFrame
from teleop_core.robot import RobotState
from teleop_core.server import _resolve_robot_asset_path
from teleop_core.telemetry import TelemetryHub
from teleop_core.types import Pose
from teleop_core.workspace import Workspace


@dataclass(frozen=True)
class _Detection:
    camera_from_board: np.ndarray
    reprojection_error: float
    corner_count: int
    overlay_rgb: np.ndarray


class JsonJointStateProvider(JointStateProvider):
    """Development provider for a fixed or externally refreshed joint JSON."""

    def __init__(self, path: Path) -> None:
        self._path = path

    def read_joint_state(self) -> dict[str, float]:
        data = json.loads(self._path.read_text())
        if not isinstance(data, dict):
            raise ValueError("joint state JSON must be an object keyed by URDF joint name")
        return {str(name): float(value) for name, value in data.items()}


class CalibrationPointCloudSource:
    """Captures all cameras, optimizes extrinsics, and emits world clouds."""

    def __init__(
        self,
        *,
        config_path: Path,
        config: HardwarePointCloudConfig,
        anchor: HardwareCameraConfig,
        optimizer: AnchoredExtrinsicOptimizer,
        joint_provider: JointStateProvider,
        kinematic_tree: UrdfKinematicTree,
        base_link: str,
        camera_link: str,
        board,
        cv2,
        min_corners: int,
        autosave: bool,
        reader_factory=default_reader_factory,
    ) -> None:
        self._config_path = Path(config_path)
        self._config = config
        self._anchor = anchor
        self._optimizer = optimizer
        self._joint_provider = joint_provider
        self._kinematic_tree = kinematic_tree
        self._base_link = base_link
        self._camera_link = camera_link
        self._board = board
        self._cv2 = cv2
        self._min_corners = int(min_corners)
        self._autosave = bool(autosave)
        self._reader_factory = reader_factory
        self._readers: list[tuple[HardwareCameraConfig, CameraPointCloudReader]] = []
        self._descriptors: dict[str, CameraDescriptor] = {}
        self.latest_joint_state: dict[str, float] = {}
        self._latest_detections: dict[str, dict] = {}
        self._latest_transforms: dict[str, list[list[float]]] = {}
        self._latest_board_poses: dict[str, list[list[float]]] = {}
        self._latest_error: str | None = None
        self._autosave_state = "disabled" if not autosave else "waiting_for_stability"
        self._last_saved_timestamp: float | None = None
        self._saved_current_stable_set = False

    async def start(self) -> None:
        if self._readers:
            return
        self._readers = [
            (camera, self._reader_factory(camera))
            for camera in self._config.cameras
        ]
        for _, reader in self._readers:
            await reader.start()
        self._descriptors = {}
        for camera, reader in self._readers:
            descriptor = reader.descriptor()
            if descriptor is not None:
                self._descriptors[camera.name] = descriptor

    async def stop(self) -> None:
        for _, reader in reversed(self._readers):
            with contextlib.suppress(Exception):
                await reader.stop()
        self._readers = []

    async def grab(self) -> Optional[PointCloudFrame]:
        if not self._readers:
            return None

        raw_frames: list[tuple[HardwareCameraConfig, PointCloudFrame]] = []
        detections: dict[str, CalibrationDetection] = {}
        detection_status: dict[str, dict] = {}

        try:
            joint_state = await asyncio.to_thread(self._joint_provider.read_joint_state)
            self.latest_joint_state = joint_state
            world_from_anchor = self._kinematic_tree.transform(
                self._base_link,
                self._camera_link,
                joint_state,
            )
            self._latest_error = None
        except Exception as exc:
            self._latest_error = repr(exc)
            return None

        for camera, reader in self._readers:
            frame = await reader.grab_camera_frame()
            if frame is not None:
                raw_frames.append((camera, frame))
            image = reader.latest_color_rgb()
            descriptor = self._descriptors.get(camera.name)
            if image is None or descriptor is None:
                detection_status[camera.name] = {
                    "detected": False,
                    "reason": "frame_or_intrinsics_unavailable",
                }
                continue
            detection = _detect_board_pose(
                self._cv2,
                self._board,
                image,
                descriptor,
                min_corners=self._min_corners,
            )
            if detection is None:
                detection_status[camera.name] = {
                    "detected": False,
                    "reason": "board_not_found",
                }
                continue
            detections[camera.name] = CalibrationDetection(
                camera_from_board=detection.camera_from_board,
                reprojection_error=detection.reprojection_error,
                corner_count=detection.corner_count,
            )
            detection_status[camera.name] = {
                "detected": True,
                "reprojection_error_px": detection.reprojection_error,
                "corner_count": detection.corner_count,
            }

        self._latest_detections = detection_status
        self._optimizer.add_frame(
            world_from_anchor_camera=world_from_anchor,
            detections=detections,
            timestamp=time.monotonic(),
        )
        self._maybe_autosave()

        transforms = self._optimizer.current_transforms(include_anchor=True)
        self._latest_transforms = {
            name: transform.tolist()
            for name, transform in transforms.items()
        }
        self._latest_board_poses = {
            name: (transforms[name] @ detection.camera_from_board).tolist()
            for name, detection in detections.items()
            if name in transforms
        }
        transformed_frames = [
            (
                replace(
                    camera,
                    world_from_camera=np.asarray(
                        transforms.get(camera.name, camera.world_from_camera),
                        dtype=np.float32,
                    ),
                    calibrated=True,
                ),
                frame,
            )
            for camera, frame in raw_frames
        ]
        return fuse_camera_frames(
            transformed_frames,
            workspace_crop=self._config.workspace_crop,
            max_points=self._config.max_points,
        )

    def dashboard_pointcloud_frame(self) -> str:
        return "world"

    def dashboard_camera_feeds(self) -> list[dict[str, object]]:
        feeds = []
        for camera, _ in self._readers:
            if not camera.urdf_link:
                continue
            feeds.append({
                "name": camera.name,
                "urdf_link": camera.urdf_link,
                "url": f"/api/cameras/{quote(camera.name, safe='')}/color.jpg",
                "width": int(camera.width),
                "height": int(camera.height),
            })
        return feeds

    def latest_color_jpeg(self, camera_name: str) -> bytes | None:
        for camera, reader in self._readers:
            if camera.name == camera_name:
                return reader.latest_color_jpeg()
        return None

    def calibration_snapshot(self) -> dict:
        status = self._optimizer.status()
        cameras = [
            {
                "name": camera.name,
                "type": camera.camera_type,
                "serial": camera.serial,
                "urdf_link": camera.urdf_link,
                "anchor": camera.name == self._anchor.name,
                "detection": self._latest_detections.get(camera.name, {
                    "detected": False,
                    "reason": "not_sampled",
                }),
            }
            for camera in self._config.cameras
        ]
        return {
            **status,
            "cameras": cameras,
            "world_from_camera": self._latest_transforms,
            "world_from_board": self._latest_board_poses,
            "autosave": {
                "enabled": self._autosave,
                "state": self._autosave_state,
                "last_saved_timestamp": self._last_saved_timestamp,
            },
            "error": self._latest_error,
        }

    def _maybe_autosave(self) -> None:
        if not self._autosave:
            return
        status = self._optimizer.status()
        if not status["all_stable"]:
            self._autosave_state = "waiting_for_stability"
            self._saved_current_stable_set = False
            return
        if self._saved_current_stable_set:
            self._autosave_state = "saved"
            return
        write_calibrated_hardware_config(
            self._config_path,
            self._optimizer.stable_transforms(include_anchor=True),
            backup=True,
        )
        self._last_saved_timestamp = time.monotonic()
        self._autosave_state = "saved"
        self._saved_current_stable_set = True


class CalibrationRobotAdapter:
    """Tiny robot-state adapter so the shared dashboard can pose the URDF."""

    def __init__(self, source: CalibrationPointCloudSource) -> None:
        self._source = source

    async def get_state(self) -> RobotState:
        joint_state = self._source.latest_joint_state
        names = [
            name for name in RC5_ARM_JOINT_NAMES
            if name in joint_state
        ]
        names.extend(name for name in sorted(joint_state) if name not in names)
        return RobotState(
            wrist_pose=Pose.identity(frame="world"),
            joint_angles=np.asarray([joint_state[name] for name in names], dtype=np.float32),
            finger_curls=np.zeros(5, dtype=np.float32),
            timestamp=time.monotonic(),
            joint_names=tuple(names),
        )


def _default_urdf() -> Path:
    return REPO_ROOT / "urdf_rc5_right_hand" / "urdf_with_simple_collisions.urdf"


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cameras", type=Path, required=True,
                    help="hardware camera config JSON to open and autosave")
    ap.add_argument("--anchor-camera", default=None,
                    help="FK-trusted camera name; defaults to --camera-link match")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--dashboard-port", type=int, default=8001)

    ap.add_argument("--urdf", type=Path, default=_default_urdf())
    ap.add_argument("--base-link", default="world")
    ap.add_argument("--camera-link", default="d405_depth_optical_frame")
    ap.add_argument("--arm-ip", default="10.10.10.10")
    ap.add_argument("--rc5-api-path", type=Path, default=None)
    ap.add_argument(
        "--joint-state-json",
        type=Path,
        default=None,
        help="offline/live joint JSON; omit to read live RC5 joints",
    )

    ap.add_argument("--squares-x", type=int, required=True)
    ap.add_argument("--squares-y", type=int, required=True)
    ap.add_argument("--square-length", type=float, required=True)
    ap.add_argument("--marker-length", type=float, required=True)
    ap.add_argument("--dictionary", default="DICT_5X5_100")

    ap.add_argument("--min-samples", type=int, default=10)
    ap.add_argument("--min-corners", type=int, default=12)
    ap.add_argument("--max-reprojection-error", type=float, default=2.0)
    ap.add_argument("--rolling-window", type=int, default=40)
    ap.add_argument("--stability-seconds", type=float, default=2.0)
    ap.add_argument("--max-pair-translation-deviation", type=float, default=0.05)
    ap.add_argument("--max-pair-rotation-deviation", type=float, default=5.0)
    ap.add_argument("--translation-stability", type=float, default=0.01)
    ap.add_argument("--rotation-stability", type=float, default=1.0)
    ap.add_argument(
        "--autosave",
        dest="autosave",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="atomically autosave stable all-camera solutions",
    )
    return ap.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    try:
        asyncio.run(run_continuous_calibration(args))
    except KeyboardInterrupt:
        pass


async def run_continuous_calibration(args: argparse.Namespace) -> None:
    cv2 = _require_cv2_aruco()
    board_spec = CharucoBoardSpec(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length=args.square_length,
        marker_length=args.marker_length,
        dictionary=args.dictionary,
    )
    board = board_spec.create_cv2_board()

    hardware_config = load_hardware_config(args.cameras)
    anchor = select_anchor_camera(
        hardware_config,
        anchor_name=args.anchor_camera,
        camera_link=args.camera_link,
    )
    target_names = tuple(
        camera.name for camera in hardware_config.cameras
        if camera.name != anchor.name
    )
    if not target_names:
        raise SystemExit("continuous calibration requires at least one non-anchor camera")

    optimizer = AnchoredExtrinsicOptimizer(
        anchor_name=anchor.name,
        target_names=target_names,
        min_samples=args.min_samples,
        rolling_window=args.rolling_window,
        min_corners=args.min_corners,
        max_reprojection_error_px=args.max_reprojection_error,
        max_pair_translation_deviation_m=args.max_pair_translation_deviation,
        max_pair_rotation_deviation_deg=args.max_pair_rotation_deviation,
        translation_stability_m=args.translation_stability,
        rotation_stability_deg=args.rotation_stability,
        stability_seconds=args.stability_seconds,
    )
    source = CalibrationPointCloudSource(
        config_path=args.cameras,
        config=hardware_config,
        anchor=anchor,
        optimizer=optimizer,
        joint_provider=_make_joint_state_provider(args),
        kinematic_tree=UrdfKinematicTree.from_file(args.urdf),
        base_link=args.base_link,
        camera_link=args.camera_link,
        board=board,
        cv2=cv2,
        min_corners=args.min_corners,
        autosave=args.autosave,
    )
    workspace = _workspace_from_config(hardware_config)
    robot = CalibrationRobotAdapter(source)
    hub = TelemetryHub(
        point_cloud_source=source,
        robot_driver=robot,
        workspace=workspace,
        urdf_url="/robot/robot.urdf",
        urdf_assets_url="/robot/assets/",
        calibration_snapshot_provider=source.calibration_snapshot,
    )

    await source.start()
    await hub.start()
    app = _make_dashboard_app(
        hub=hub,
        source=source,
        static_dir=REPO_ROOT / "webxr_app" / "dashboard_static",
        urdf_path=args.urdf,
        robot_assets_root=args.urdf.parent,
    )
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.dashboard_port)
    try:
        await site.start()
        print(f"[calib] anchor camera: {anchor.name}")
        print(f"[calib] target cameras: {', '.join(target_names)}")
        print(f"[calib] dashboard on http://{args.host}:{args.dashboard_port}")
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()
        await hub.stop()
        await source.stop()


def _make_joint_state_provider(args: argparse.Namespace) -> JointStateProvider:
    if args.joint_state_json is not None:
        return JsonJointStateProvider(args.joint_state_json)
    return RC5JointStateReader(
        arm_ip=args.arm_ip,
        rc5_api_path=args.rc5_api_path,
    )


def _workspace_from_config(config: HardwarePointCloudConfig) -> Workspace:
    if config.workspace_crop is not None:
        min_corner, max_corner = config.workspace_crop
        return Workspace(
            min_corner=np.asarray(min_corner, dtype=np.float32),
            max_corner=np.asarray(max_corner, dtype=np.float32),
        )
    return Workspace(
        min_corner=np.array([-0.5, -0.5, 0.0], dtype=np.float32),
        max_corner=np.array([0.8, 0.8, 1.2], dtype=np.float32),
    )


def _make_dashboard_app(
    *,
    hub: TelemetryHub,
    source: CalibrationPointCloudSource,
    static_dir: Path,
    urdf_path: Path,
    robot_assets_root: Path,
) -> web.Application:
    app = web.Application()

    async def snapshot(_request) -> web.Response:
        return web.json_response(hub.snapshot())

    async def camera_color(request) -> web.Response:
        image = source.latest_color_jpeg(request.match_info["name"])
        if image is None:
            return web.Response(status=404, text="camera frame not ready")
        return web.Response(
            body=image,
            content_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    async def robot_asset(request) -> web.StreamResponse:
        try:
            path = _resolve_robot_asset_path(robot_assets_root, request.match_info["tail"])
        except ValueError:
            return web.Response(status=404)
        if not path.exists() or not path.is_file():
            return web.Response(status=404)
        return web.FileResponse(path)

    async def dashboard_ws(request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        await ws.send_str(json.dumps(hub.snapshot()))

        async def json_loop() -> None:
            while not ws.closed:
                try:
                    await ws.send_str(json.dumps(hub.snapshot()))
                except ConnectionResetError:
                    break
                await asyncio.sleep(1.0 / 20.0)

        async def cloud_loop() -> None:
            last_sequence = 0
            while not ws.closed:
                cloud = await hub.wait_for_pointcloud(
                    after_sequence=last_sequence,
                    timeout=1.0,
                )
                if cloud is None:
                    continue
                last_sequence = cloud.sequence
                try:
                    await ws.send_bytes(cloud.payload)
                except ConnectionResetError:
                    break

        tasks = [
            asyncio.create_task(json_loop(), name="calibration_dashboard_json_loop"),
            asyncio.create_task(cloud_loop(), name="calibration_dashboard_cloud_loop"),
        ]
        try:
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
                if msg.type == WSMsgType.TEXT:
                    await ws.send_str(json.dumps({
                        "type": "error",
                        "message": "calibration dashboard is read-only",
                    }))
        finally:
            for task in tasks:
                task.cancel()
            for task in tasks:
                with contextlib.suppress(asyncio.CancelledError):
                    await task
        return ws

    app.router.add_get("/ws", dashboard_ws)
    app.router.add_get("/api/snapshot", snapshot)
    app.router.add_get("/api/cameras/{name}/color.jpg", camera_color)
    app.router.add_get("/robot/robot.urdf", lambda _request: web.FileResponse(urdf_path))
    app.router.add_get("/robot/assets/{tail:.*}", robot_asset)
    app.router.add_get("/", lambda _request: web.FileResponse(static_dir / "index.html"))
    app.router.add_static("/", path=str(static_dir), show_index=False)
    return app


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
