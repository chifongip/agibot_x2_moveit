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


def test_state_topic_prefix_is_normalized_and_isolated():
    topics = MODULE.state_topics("/x2_replay/")
    assert set(topics) == {
        "/x2_replay/aima/hal/joint/leg/state",
        "/x2_replay/aima/hal/joint/waist/state",
        "/x2_replay/aima/hal/joint/arm/state",
        "/x2_replay/aima/hal/joint/head/state",
    }
    assert MODULE.state_topics("") == MODULE.STATE_TOPICS


def test_valid_partial_arm_target():
    positions = MODULE.validate_positions(
        {"positions": {"left_shoulder_pitch_joint": 0.35, "right_elbow_joint": -0.87}}
    )
    assert positions == {
        "left_shoulder_pitch_joint": 0.35,
        "right_elbow_joint": -0.87,
    }


def test_recorded_initial_state_must_cover_all_x2_joints(tmp_path):
    snapshot = tmp_path / "snapshot.yaml"
    snapshot.write_text(
        "joint_positions:\n"
        + "".join(
            f"  {name}: {index / 100.0}\n"
            for index, name in enumerate(MODULE.ALL_JOINT_NAMES)
        ),
        encoding="utf-8",
    )

    positions = MODULE.load_initial_positions(snapshot)

    assert positions == {
        name: index / 100.0 for index, name in enumerate(MODULE.ALL_JOINT_NAMES)
    }


def test_recorded_initial_state_rejects_missing_joint(tmp_path):
    snapshot = tmp_path / "incomplete.yaml"
    snapshot.write_text("joint_positions: {}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must match X2 exactly"):
        MODULE.load_initial_positions(snapshot)


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
