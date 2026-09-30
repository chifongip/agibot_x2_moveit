"""Replay a pre-task capture with fake HAL feedback and frozen tag observations."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    snapshot = LaunchConfiguration("snapshot")
    return LaunchDescription([
        DeclareLaunchArgument("snapshot", description="Absolute pre-task snapshot YAML path."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("allow_execution", default_value="false"),
        DeclareLaunchArgument("disable_table_collision", default_value="false"),
        DeclareLaunchArgument("zmq_endpoint", default_value="tcp://*:8559"),
        DeclareLaunchArgument("fake_zmq_endpoint", default_value="tcp://127.0.0.1:8559"),
        Node(
            package="agibot_x2_manipulation", executable="capture_task_snapshot",
            name="snapshot_tag_replay", output="screen",
            arguments=["--replay", snapshot],
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
            }.items(),
        ),
    ])
