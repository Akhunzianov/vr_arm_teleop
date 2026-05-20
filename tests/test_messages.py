import json

from teleop_core.messages import HandStateMsg, decode


def test_hand_state_decode_has_head_pose_defaults():
    msg = decode(json.dumps({"type": "hand", "valid": True}))

    assert isinstance(msg, HandStateMsg)
    assert msg.head_position == (0.0, 0.0, 0.0)
    assert msg.head_orientation == (0.0, 0.0, 0.0, 1.0)
    assert msg.head_valid is False


def test_hand_state_decode_accepts_head_pose_fields():
    msg = decode(json.dumps({
        "type": "hand",
        "valid": True,
        "wrist_position": [0.1, 0.2, 0.3],
        "wrist_orientation": [0.0, 0.0, 0.0, 1.0],
        "head_position": [1.0, 1.5, -0.2],
        "head_orientation": [0.0, 0.707, 0.0, 0.707],
        "head_valid": True,
    }))

    assert msg.head_position == (1.0, 1.5, -0.2)
    assert msg.head_orientation == (0.0, 0.707, 0.0, 0.707)
    assert msg.head_valid is True
