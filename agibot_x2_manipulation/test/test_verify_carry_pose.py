import importlib.util
import math
from pathlib import Path

import pytest
import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "verify_carry_pose.py"
SPEC = importlib.util.spec_from_file_location("verify_carry_pose", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(x=0.25, y=0.0, z=0.34, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return [x, y, z, qx, qy, qz, qw]


def snapshot(profile_id="grey_box", instance_id="tag:180"):
    return {
        "joint_positions": {name: 0.0 for name in MODULE.EXPECTED_JOINTS},
        "visible_box_states": [
            {
                "instance_id": instance_id,
                "profile_id": profile_id,
                "frame_id": "base_link",
                "pose": {
                    "position": {"x": 0.31, "y": 0.0, "z": 0.18},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
        ],
    }


def test_normalize_pose_normalizes_quaternion_and_rejects_zero_quaternion():
    assert MODULE.normalize_pose([0.25, 0.0, 0.34, 0.0, 0.0, 0.0, 2.0]) == pose()
    with pytest.raises(ValueError, match="quaternion must be nonzero"):
        MODULE.normalize_pose([0.25, 0.0, 0.34, 0.0, 0.0, 0.0, 0.0])


def test_snapshot_selection_requires_an_unambiguous_box_in_base_link():
    selected = MODULE.select_snapshot_box(snapshot(), "grey_box")

    assert selected["instance_id"] == "tag:180"
    assert selected["profile_id"] == "grey_box"
    assert selected["pose"] == [0.31, 0.0, 0.18, 0.0, 0.0, 0.0, 1.0]

    ambiguous = snapshot()
    ambiguous["visible_box_states"].append(
        {
            **ambiguous["visible_box_states"][0],
            "instance_id": "tag:181",
        }
    )
    with pytest.raises(ValueError, match="specify --instance-id"):
        MODULE.select_snapshot_box(ambiguous, "grey_box")


def test_snapshot_selection_rejects_incomplete_joint_maps_and_non_base_box_frames():
    incomplete = snapshot()
    incomplete["joint_positions"].pop("head_pitch_joint")
    with pytest.raises(ValueError, match="must match X2 exactly"):
        MODULE.joint_positions_from_snapshot(incomplete)

    wrong_frame = snapshot()
    wrong_frame["visible_box_states"][0]["frame_id"] = "odom"
    with pytest.raises(ValueError, match="frame_id must be 'base_link'"):
        MODULE.select_snapshot_box(wrong_frame, "grey_box")


def test_temporary_profiles_overrides_only_requested_profile_carry_a(tmp_path):
    profiles_file = tmp_path / "profiles.yaml"
    original = {
        "/**": {
            "ros__parameters": {
                "box_profiles": {
                    "grey_box": {
                        "carry_pose_a": pose(),
                        "carry_pose_b": pose(0.30),
                    },
                    "small_carton": {"carry_pose_a": pose(0.20)},
                }
            }
        }
    }
    profiles_file.write_text(yaml.safe_dump(original), encoding="utf-8")

    overridden = MODULE.temporary_profiles(profiles_file, "grey_box", pose(0.28, 0.02))
    profiles = overridden["/**"]["ros__parameters"]["box_profiles"]

    assert profiles["grey_box"]["carry_pose_a"] == pose(0.28, 0.02)
    assert profiles["grey_box"]["carry_pose_b"] == pose(0.30)
    assert profiles["small_carton"]["carry_pose_a"] == pose(0.20)


def test_classification_distinguishes_exact_target_from_adaptive_fallback():
    requested = pose()
    same_rotation_opposite_sign = pose(qw=-1.0)

    outcome, position_error, orientation_error = MODULE.classification(
        True, requested, same_rotation_opposite_sign, 0.001, math.radians(1.0)
    )
    assert outcome == "exact_feasible"
    assert position_error == 0.0
    assert orientation_error == 0.0

    outcome, position_error, orientation_error = MODULE.classification(
        True, requested, pose(0.25, 0.02), 0.001, math.radians(1.0)
    )
    assert outcome == "adaptive_fallback"
    assert position_error == pytest.approx(0.02)
    assert orientation_error == 0.0

    outcome, position_error, orientation_error = MODULE.classification(
        False, requested, requested, 0.001, math.radians(1.0)
    )
    assert outcome == "infeasible"
    assert position_error is None
    assert orientation_error is None
