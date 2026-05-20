import importlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from teleop_backends.camera_calibration import (
    CameraDescriptor,
    CalibrationObservation,
    CharucoBoardSpec,
    RealSenseColorFeed,
    UnavailableJointStateProvider,
    UrdfKinematicTree,
    ZedColorFeed,
    build_hardware_camera_config,
    estimate_two_camera_extrinsics,
    invert_transform,
    rotation_matrix_from_axis_angle,
    transform_from_rt,
)
from teleop_backends.pointcloud.hardware import load_hardware_config


def test_estimate_two_camera_extrinsics_chains_external_camera_to_world():
    world_from_arm_camera = transform_from_rt(
        rotation_matrix_from_axis_angle([0, 0, 1], 0.3),
        [0.4, -0.2, 0.8],
    )
    arm_camera_from_external_camera = transform_from_rt(
        rotation_matrix_from_axis_angle([0, 1, 0], -0.25),
        [0.15, 0.03, -0.08],
    )
    external_camera_from_arm_camera = invert_transform(arm_camera_from_external_camera)

    observations = []
    for i in range(12):
        arm_camera_from_board = transform_from_rt(
            rotation_matrix_from_axis_angle([1, 0.2, 0.1], 0.05 * i),
            [0.02 * i, -0.01 * i, 0.7 + 0.03 * i],
        )
        observations.append(
            CalibrationObservation(
                arm_camera_from_board=arm_camera_from_board,
                external_camera_from_board=(
                    external_camera_from_arm_camera @ arm_camera_from_board
                ),
                arm_reprojection_error=0.15,
                external_reprojection_error=0.2,
                corner_count=30,
            )
        )

    result = estimate_two_camera_extrinsics(
        observations,
        world_from_arm_camera=world_from_arm_camera,
        min_samples=10,
    )

    assert result.accepted_samples == 12
    assert result.rejected_samples == 0
    assert np.allclose(
        result.arm_camera_from_external_camera,
        arm_camera_from_external_camera,
        atol=1e-6,
    )
    assert np.allclose(
        result.world_from_external_camera,
        world_from_arm_camera @ arm_camera_from_external_camera,
        atol=1e-6,
    )


def test_build_hardware_camera_config_writes_runtime_compatible_extrinsics(tmp_path):
    world_from_arm_camera = transform_from_rt(np.eye(3), [0.1, 0.2, 0.3])
    world_from_external_camera = transform_from_rt(np.eye(3), [0.4, 0.5, 0.6])
    result = estimate_two_camera_extrinsics(
        [
            CalibrationObservation(
                arm_camera_from_board=np.eye(4),
                external_camera_from_board=transform_from_rt(np.eye(3), [-0.3, -0.3, -0.3]),
                arm_reprojection_error=0.1,
                external_reprojection_error=0.1,
                corner_count=24,
            )
            for _ in range(10)
        ],
        world_from_arm_camera=world_from_arm_camera,
        min_samples=10,
    )
    assert np.allclose(result.world_from_external_camera, world_from_external_camera)

    config = build_hardware_camera_config(
        result,
        arm_camera=CameraDescriptor(
            name="arm-cam",
            camera_type="zed2i",
            serial="zed-serial",
            width=1280,
            height=720,
            fps=30,
            camera_matrix=[[700, 0, 640], [0, 700, 360], [0, 0, 1]],
            distortion=[0, 0, 0, 0, 0],
        ),
        external_camera=CameraDescriptor(
            name="external-cam",
            camera_type="realsense",
            serial="rs-serial",
            width=640,
            height=480,
            fps=30,
            camera_matrix=[[610, 0, 320], [0, 610, 240], [0, 0, 1]],
            distortion=[0, 0, 0, 0, 0],
        ),
    )
    path = tmp_path / "cameras.json"
    path.write_text(json.dumps(config))

    loaded = load_hardware_config(path)

    assert [camera.name for camera in loaded.cameras] == ["arm-cam", "external-cam"]
    assert all(camera.calibrated for camera in loaded.cameras)
    assert np.allclose(loaded.cameras[0].world_from_camera, world_from_arm_camera)
    assert np.allclose(loaded.cameras[1].world_from_camera, world_from_external_camera)
    assert config["cameras"][0]["extrinsic_world_from_cam"] == config["cameras"][0]["world_from_camera"]


def test_charuco_board_spec_validates_dimensions_and_dictionary():
    spec = CharucoBoardSpec(
        squares_x=7,
        squares_y=5,
        square_length=0.035,
        marker_length=0.026,
        dictionary="DICT_5X5_100",
    )

    assert spec.squares_x == 7
    assert spec.squares_y == 5

    with pytest.raises(ValueError, match="marker_length"):
        CharucoBoardSpec(
            squares_x=7,
            squares_y=5,
            square_length=0.035,
            marker_length=0.04,
            dictionary="DICT_5X5_100",
        )


def test_urdf_fk_computes_base_to_camera_link(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """
        <robot name="test">
          <link name="base"/>
          <link name="joint_link"/>
          <link name="camera_optical"/>
          <joint name="pan" type="revolute">
            <parent link="base"/>
            <child link="joint_link"/>
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="camera_mount" type="fixed">
            <parent link="joint_link"/>
            <child link="camera_optical"/>
            <origin xyz="0 2 0" rpy="0 0 0"/>
          </joint>
        </robot>
        """
    )

    tree = UrdfKinematicTree.from_file(urdf)
    base_from_camera = tree.transform("base", "camera_optical", {"pan": math.pi / 2})

    assert np.allclose(base_from_camera[:3, 3], [-1, 0, 0], atol=1e-6)
    assert np.allclose(
        base_from_camera[:3, :3],
        rotation_matrix_from_axis_angle([0, 0, 1], math.pi / 2),
        atol=1e-6,
    )


def test_canonical_urdf_resolves_world_to_d405_depth_optical_frame():
    urdf = (
        Path(__file__).resolve().parents[1]
        / "urdf_rc5_right_hand"
        / "urdf_with_simple_collisions.urdf"
    )

    tree = UrdfKinematicTree.from_file(urdf)
    base_from_camera = tree.transform(
        "world",
        "d405_depth_optical_frame",
        {f"joint{i}": 0.0 for i in range(6)},
    )

    assert base_from_camera.shape == (4, 4)
    assert np.all(np.isfinite(base_from_camera))


def test_missing_camera_sdks_are_reported_only_when_feeds_start(monkeypatch):
    def missing_import(name):
        if name in {"pyrealsense2", "pyzed.sl"}:
            raise ModuleNotFoundError(name)
        return importlib.import_module(name)

    monkeypatch.setattr(importlib, "import_module", missing_import)

    rs_feed = RealSenseColorFeed(serial="rs-serial", width=640, height=480, fps=30)
    zed_feed = ZedColorFeed(serial="123456", resolution="HD720", fps=30)

    with pytest.raises(RuntimeError, match="pyrealsense2"):
        rs_feed.start()
    with pytest.raises(RuntimeError, match="pyzed.sl"):
        zed_feed.start()


def test_placeholder_joint_state_provider_fails_loudly():
    provider = UnavailableJointStateProvider("install the future arm SDK")

    with pytest.raises(RuntimeError, match="future arm SDK"):
        provider.read_joint_state()
