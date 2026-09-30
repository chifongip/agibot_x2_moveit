"""Replay a pre-task capture with fake HAL feedback and frozen tag observations."""

import os
from pathlib import Path
import uuid

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_manipulation_state(context):
    """Seed only a new, isolated replay file; never modify the robot's state file."""
    with open(LaunchConfiguration("snapshot").perform(context), encoding="utf-8") as stream:
        snapshot = yaml.safe_load(stream)
    manipulation = snapshot.get("manipulation", {})
    record = manipulation.get("persisted_record")
    if not record:
        if snapshot.get("capture", {}).get("task_kind") == "place":
            raise ValueError("Place snapshot has no persisted held-object geometry")
        return []
    if manipulation.get("state") != 2 or not record.startswith("VERSION 4\nSTATE HOLDING\n"):
        raise ValueError("invalid held-object recovery record")
    path = Path(LaunchConfiguration("manipulation_state_file").perform(context)).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(record)
    return [LogInfo(msg=(
        "Loaded held-object geometry. After startup, restore the simulated Place state with: "
        "ros2 service call /recover_manipulation_state "
        "agibot_x2_manipulation_msgs/srv/RecoverManipulationState '{requested_state: 1}'"
    ))]


def generate_launch_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    snapshot = LaunchConfiguration("snapshot")
    with open(os.path.join(share, "config", "box_manipulation.yaml"), encoding="utf-8") as stream:
        server_params = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]
    table_topic = server_params["table_tag_detections_topic"]
    return LaunchDescription([
        DeclareLaunchArgument("snapshot", description="Absolute pre-task snapshot YAML path."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("allow_execution", default_value="false"),
        DeclareLaunchArgument("disable_table_collision", default_value="false"),
        DeclareLaunchArgument("zmq_endpoint", default_value="tcp://*:8559"),
        DeclareLaunchArgument("fake_zmq_endpoint", default_value="tcp://127.0.0.1:8559"),
        DeclareLaunchArgument("manipulation_state_file",
                              default_value=f"/tmp/x2_snapshot_state_{uuid.uuid4().hex}"),
        OpaqueFunction(function=prepare_manipulation_state),
        Node(
            package="agibot_x2_manipulation", executable="capture_task_snapshot",
            name="snapshot_tag_replay", output="screen",
            arguments=["--replay", snapshot, "--detections-topic", "/detections",
                       "--detections-topic", table_topic],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, "launch", "recorded_planning_failure.launch.py"
            )),
            launch_arguments={
                "joint_snapshot": snapshot,
                "initial_arm_command_mode": "measured",
                "use_dummy_apriltag": "false",
                "disable_table_collision": LaunchConfiguration("disable_table_collision"),
                "allow_execution": LaunchConfiguration("allow_execution"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "zmq_endpoint": LaunchConfiguration("zmq_endpoint"),
                "fake_zmq_endpoint": LaunchConfiguration("fake_zmq_endpoint"),
                "posture_zmq_enabled": "false",
                "manipulation_state_file": LaunchConfiguration("manipulation_state_file"),
            }.items(),
        ),
    ])
