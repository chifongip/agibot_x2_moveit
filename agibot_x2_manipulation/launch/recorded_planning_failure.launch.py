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
    posture_zmq_enabled = LaunchConfiguration("posture_zmq_enabled")
    posture_zmq_endpoint = LaunchConfiguration("posture_zmq_endpoint")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_execution")
    motion_planning_mode = LaunchConfiguration("motion_planning_mode")
    manipulation_state_file = LaunchConfiguration("manipulation_state_file")
    default_joint_snapshot = os.path.join(
        manipulation_share, "config", "recorded_planning_failure_joint_state.yaml"
    )
    default_tag_snapshot = os.path.join(
        manipulation_share, "config", "recorded_planning_failure_dummy_apriltag.yaml"
    )

    joint_snapshot = LaunchConfiguration("joint_snapshot")
    tag_snapshot = LaunchConfiguration("tag_snapshot")

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_arm_command_mode", default_value="ready"),
            DeclareLaunchArgument("joint_snapshot", default_value=default_joint_snapshot),
            DeclareLaunchArgument("tag_snapshot", default_value=default_tag_snapshot),
            DeclareLaunchArgument("use_dummy_apriltag", default_value="true"),
            DeclareLaunchArgument(
                "disable_table_collision", default_value=LaunchConfiguration("use_dummy_apriltag")
            ),
            DeclareLaunchArgument("zmq_endpoint", default_value="tcp://*:8559"),
            DeclareLaunchArgument(
                "posture_zmq_enabled",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Keep the replay isolated from RoboJuDo posture control. "
                    "The production box_pick_place launch enables it by default."
                ),
            ),
            DeclareLaunchArgument("posture_zmq_endpoint", default_value="tcp://*:8557"),
            DeclareLaunchArgument(
                "fake_zmq_endpoint", default_value="tcp://127.0.0.1:8559"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("phase_retry_attempts", default_value="3"),
            DeclareLaunchArgument("phase_retry_timeout", default_value="30.0"),
            DeclareLaunchArgument("phase_retry_delay", default_value="0.5"),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="true",
                description=(
                    "Allow non-plan-only actions in this isolated recorded-state "
                    "simulation"
                ),
            ),
            DeclareLaunchArgument(
                "motion_planning_mode",
                default_value="closed_chain",
                choices=["closed_chain", "pose_to_pose"],
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
                    "initial_arm_command_mode": LaunchConfiguration("initial_arm_command_mode"),
                    "zmq_endpoint": zmq_endpoint,
                    "posture_zmq_enabled": posture_zmq_enabled,
                    "posture_zmq_endpoint": posture_zmq_endpoint,
                    "use_rviz": use_rviz,
                    "use_apriltag": "false",
                    "use_dummy_apriltag": LaunchConfiguration("use_dummy_apriltag"),
                    "disable_table_collision": LaunchConfiguration("disable_table_collision"),
                    "start_table_tag_detector": "false",
                    "dummy_tag_params_file": tag_snapshot,
                    "perception_3d_source": "none",
                    "leg_state_topic": "/x2_replay/aima/hal/joint/leg/state",
                    "waist_state_topic": "/x2_replay/aima/hal/joint/waist/state",
                    "arm_state_topic": "/x2_replay/aima/hal/joint/arm/state",
                    "head_state_topic": "/x2_replay/aima/hal/joint/head/state",
                    "allow_execution": allow_execution,
                    "motion_planning_mode": motion_planning_mode,
                    "phase_retry_attempts": LaunchConfiguration("phase_retry_attempts"),
                    "phase_retry_timeout": LaunchConfiguration("phase_retry_timeout"),
                    "phase_retry_delay": LaunchConfiguration("phase_retry_delay"),
                    "manipulation_state_file": manipulation_state_file,
                }.items(),
            ),
        ]
    )
