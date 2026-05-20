import argparse
import json
import sys
import types

import pytest

sys.modules.setdefault("pybullet", types.SimpleNamespace())

from teleop_backends.pointcloud.hardware import HardwarePointCloudSource
from webxr_app.__main__ import _make_pc_source


def _write_config(tmp_path):
    path = tmp_path / "cameras.json"
    path.write_text(
        json.dumps(
            {
                "cameras": [
                    {
                        "name": "rs-front",
                        "type": "realsense",
                        "serial": "123",
                        "calibrated": False,
                    }
                ]
            }
        )
    )
    return path


def test_hardware_pc_backend_uses_configured_fused_source(tmp_path):
    args = argparse.Namespace(cameras=_write_config(tmp_path))

    source = _make_pc_source("hardware", args)

    assert isinstance(source, HardwarePointCloudSource)


def test_hardware_pc_backend_requires_cameras_config():
    args = argparse.Namespace(cameras=None)

    with pytest.raises(SystemExit, match="--pc-backend hardware requires --cameras"):
        _make_pc_source("hardware", args)


def test_realsense_pc_backend_uses_legacy_realsense_wrapper(tmp_path):
    args = argparse.Namespace(cameras=_write_config(tmp_path))

    source = _make_pc_source("realsense", args)

    assert isinstance(source, HardwarePointCloudSource)
