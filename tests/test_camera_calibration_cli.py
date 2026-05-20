from pathlib import Path
import subprocess
import sys

from scripts.calibrate_two_cameras_charuco import parse_args


def test_calibration_cli_parses_board_camera_and_urdf_options():
    args = parse_args(
        [
            "--arm-camera",
            "zed",
            "--arm-serial",
            "12345",
            "--external-camera",
            "realsense",
            "--external-serial",
            "67890",
            "--urdf",
            "robot.urdf",
            "--base-link",
            "base",
            "--camera-link",
            "camera_optical",
            "--squares-x",
            "7",
            "--squares-y",
            "5",
            "--square-length",
            "0.035",
            "--marker-length",
            "0.026",
            "--dictionary",
            "DICT_5X5_100",
            "--output",
            "config/cameras.json",
        ]
    )

    assert args.arm_camera == "zed"
    assert args.external_camera == "realsense"
    assert args.urdf == Path("robot.urdf")
    assert args.base_link == "base"
    assert args.camera_link == "camera_optical"
    assert args.squares_x == 7
    assert args.square_length == 0.035
    assert args.output == Path("config/cameras.json")


def test_calibration_script_help_runs_as_direct_file():
    result = subprocess.run(
        [sys.executable, "scripts/calibrate_two_cameras_charuco.py", "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "--arm-camera" in result.stdout
