"""Replay the captured X2 joint and box snapshot in an isolated ZMQ simulation."""

import os
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manipulation_share = get_package_share_directory("agibot_x2_manipulation")
    zmq_endpoint = LaunchConfiguration("zmq_endpoint")
    fake_zmq_endpoint = LaunchConfiguration("fake_zmq_endpoint")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_execution")
    manipulation_state_file = LaunchConfiguration("manipulation_state_file")
    joint_snapshot = os.path.join(
        manipulation_share, "config", "recorded_planning_failure_joint_state.yaml"
    )
    tag_snapshot = os.path.join(
        manipulation_share, "config", "recorded_planning_failure_dummy_apriltag.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("zmq_endpoint", default_value="tcp://*:8559"),
            DeclareLaunchArgument(
                "fake_zmq_endpoint", default_value="tcp://127.0.0.1:8559"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="true",
                description=(
                    "Allow non-plan-only actions in this isolated recorded-state "
                    "simulation"
                ),
            ),
            DeclareLaunchArgument(
                "manipulation_state_file",
                default_value=(
                    f"/tmp/agibot_x2_recorded_replay_state_{uuid.uuid4().hex}"
                ),
                description="Per-launch recovery state for the isolated replay.",
            ),
            Node(
                package="agibot_x2_ros2_control",
                executable="fake_zmq_joint_states",
                name="recorded_x2_joint_states",
                output="screen",
                arguments=[
                    "--endpoint",
                    fake_zmq_endpoint,
                    "--initial-state-file",
                    joint_snapshot,
                    "--state-topic-prefix",
                    "/x2_replay",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(manipulation_share, "launch", "box_pick_place.launch.py")
                ),
                launch_arguments={
                    "command_transport": "zmq",
                    "zmq_endpoint": zmq_endpoint,
                    "use_rviz": use_rviz,
                    "use_apriltag": "false",
                    "use_dummy_apriltag": "true",
                    "dummy_tag_params_file": tag_snapshot,
                    "perception_3d_source": "none",
                    "leg_state_topic": "/x2_replay/aima/hal/joint/leg/state",
                    "waist_state_topic": "/x2_replay/aima/hal/joint/waist/state",
                    "arm_state_topic": "/x2_replay/aima/hal/joint/arm/state",
                    "head_state_topic": "/x2_replay/aima/hal/joint/head/state",
                    "allow_execution": allow_execution,
                    "manipulation_state_file": manipulation_state_file,
                }.items(),
            ),
        ]
    )
