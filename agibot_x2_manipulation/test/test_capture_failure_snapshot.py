import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).parents[1] / "scripts" / "capture_failure_snapshot.py"
SPEC = importlib.util.spec_from_file_location("capture_failure_snapshot", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def stamp(seconds=12, nanoseconds=34):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def test_tag_id_from_instance_accepts_only_stable_tag_instance_ids():
    assert MODULE.tag_id_from_instance("tag:180") == 180
    assert MODULE.tag_id_from_instance("tag:001") == 1
    assert MODULE.tag_id_from_instance("legacy") is None
    assert MODULE.tag_id_from_instance("tag:-1") is None
    assert MODULE.tag_id_from_instance("tag:one") is None


def test_joint_state_snapshot_keeps_only_finite_positions_and_sorts_names():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="base_link", stamp=stamp()),
        name=["right_elbow_joint", "left_elbow_joint", "bad_joint"],
        position=[-0.87, 0.42, math.nan],
    )

    snapshot = MODULE.joint_state_to_dict(message)

    assert snapshot["frame_id"] == "base_link"
    assert snapshot["stamp"] == {"sec": 12, "nanosec": 34}
    assert snapshot["joint_positions"] == {
        "left_elbow_joint": 0.42,
        "right_elbow_joint": -0.87,
    }
    assert snapshot["invalid_position_joints"] == ["bad_joint"]


def test_parse_arguments_uses_safe_default_topics_and_accepts_repeated_tags(tmp_path):
    output = tmp_path / "failure.yaml"

    arguments = MODULE.parse_arguments(
        ["capture_failure_snapshot", "--output", str(output), "--tag-id", "0", "--tag-id", "180"]
    )

    assert arguments.output == str(output)
    assert arguments.tag_id == [0, 180]
    assert arguments.detections_topic is None
    assert arguments.action_status_topic is None
    assert MODULE.DEFAULT_ACTION_STATUS_TOPICS == (
        "/pick_box/_action/status",
        "/place_box/_action/status",
        "/pick_place/_action/status",
        "/move_carry_pose/_action/status",
    )
