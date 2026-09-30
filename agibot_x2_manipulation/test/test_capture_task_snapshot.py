import copy
import importlib.util
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location(
    "capture_task_snapshot", SCRIPTS / "capture_task_snapshot.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def snapshot():
    return {
        "robot": {"joint_state": {
            "joint_positions": {name: 0.1 for name in MODULE.X2_JOINT_NAMES},
            "stamp": {"sec": 10, "nanosec": 0},
        }},
        "visible_box_states": [{
            "instance_id": "tag:0", "stamp": {"sec": 10, "nanosec": 0},
            "pose": {"position": dict(x=0.4, y=0.0, z=0.2),
                     "orientation": dict(x=0.0, y=0.0, z=0.0, w=1.0)},
        }],
        "tags": [{"tag_id": tag, "pose": {"transform": {
            "translation": dict(x=0.4, y=0.0, z=0.3),
            "rotation": dict(x=0.1, y=0.2, z=0.3, w=0.9),
        }}} for tag in [0, 9]],
    }


def ready(data):
    return MODULE.capture_readiness(data, {0, 9}, "tag:0", 10.5, 1.0)


def test_accepts_complete_fresh_snapshot():
    assert len(MODULE.X2_JOINT_NAMES) == 31
    assert ready(snapshot()) is None


@pytest.mark.parametrize("mutation", [
    lambda data: data["robot"]["joint_state"]["joint_positions"].pop("head_yaw_joint"),
    lambda data: data["visible_box_states"].clear(),
    lambda data: data["tags"].pop(),
    lambda data: data["tags"][0]["pose"].clear(),
    lambda data: data["robot"]["joint_state"]["stamp"].update(sec=5),
    lambda data: data["robot"]["joint_state"]["stamp"].update(sec=15),
    lambda data: data["visible_box_states"][0]["stamp"].update(sec=5),
    lambda data: data["visible_box_states"][0]["pose"]["position"].update(x=float("nan")),
    lambda data: data["tags"][0]["pose"]["transform"]["rotation"].update(
        x=0.0, y=0.0, z=0.0, w=0.0),
    lambda data: data["robot"].update(base_pose={"error": "missing TF"}),
])
def test_refuses_incomplete_or_stale_capture(mutation):
    data = copy.deepcopy(snapshot())
    mutation(data)
    assert ready(data) is not None


@pytest.mark.parametrize("option,value", [("--timeout", "nan"), ("--max-age", "0"),
                                          ("--tag-id", "-1")])
def test_rejects_invalid_options(option, value):
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(["capture_task_snapshot", "--output", "/tmp/test.yaml",
                                option, value])


def test_capture_defaults_include_pickup_and_table_tags():
    args = MODULE.parse_arguments(["capture_task_snapshot", "--output", "/tmp/test.yaml"])
    assert args.tag_id == [0, 9]
    assert args.capture_once is False


def test_replay_preserves_all_quaternion_components():
    from apriltag_msgs.msg import AprilTagDetectionArray
    from geometry_msgs.msg import TransformStamped
    from types import SimpleNamespace

    transforms = []
    detections = []
    replay = MODULE.SnapshotTagReplay.__new__(MODULE.SnapshotTagReplay)
    replay.tags = [{"tag_id": 9, "pose": {"transform": {
        "parent_frame": "base_link", "child_frame": "tag9",
        "translation": dict(x=0.4, y=-0.2, z=0.3),
        "rotation": dict(x=0.1, y=0.2, z=0.3, w=0.9),
    }}}]
    replay.planning_frame = "base_link"
    stamp = TransformStamped().header.stamp
    stamp.sec = 50
    replay.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    replay.broadcaster = SimpleNamespace(sendTransform=transforms.extend)
    replay.publisher = SimpleNamespace(publish=detections.append)
    replay.publish()
    assert transforms[0].header.stamp.sec == 50
    rotation = transforms[0].transform.rotation
    assert [rotation.x, rotation.y, rotation.z, rotation.w] == [0.1, 0.2, 0.3, 0.9]
    assert isinstance(detections[0], AprilTagDetectionArray)
    assert detections[0].detections[0].id == 9


def test_replay_launch_holds_measured_arm_state(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    from launch.actions import IncludeLaunchDescription

    path = SCRIPTS.parent / "launch" / "recorded_task_snapshot.launch.py"
    spec = importlib.util.spec_from_file_location("recorded_task_launch", path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    description = launch_module.generate_launch_description()
    includes = [action for action in description.entities
                if isinstance(action, IncludeLaunchDescription)]
    args = dict(includes[0].launch_arguments)
    assert args["initial_arm_command_mode"] == "measured"
    assert args["use_dummy_apriltag"] == "false"
    assert args["posture_zmq_enabled"] == "false"
