import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/fake_zmq_joint_states.py"
SPEC = importlib.util.spec_from_file_location("fake_zmq_joint_states", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_joint_groups_cover_x2_once():
    names = [name for group in MODULE.STATE_TOPICS.values() for name in group]
    assert len(names) == 31
    assert len(set(names)) == 31
    assert set(names) == set(MODULE.LOCOMANIPULATION_POSITIONS)


def test_valid_partial_arm_target():
    positions = MODULE.validate_positions(
        {"positions": {"left_shoulder_pitch_joint": 0.35, "right_elbow_joint": -0.87}}
    )
    assert positions == {
        "left_shoulder_pitch_joint": 0.35,
        "right_elbow_joint": -0.87,
    }


@pytest.mark.parametrize(
    "message",
    [
        {},
        {"positions": {}},
        {"positions": {"waist_yaw_joint": 0.1}},
        {"positions": {"left_elbow_joint": True}},
        {"positions": {"left_elbow_joint": math.nan}},
    ],
)
def test_invalid_target_is_rejected(message):
    with pytest.raises(ValueError):
        MODULE.validate_positions(message)
