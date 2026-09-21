import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.util
from moveit_configs_utils import MoveItConfigsBuilder
import pytest
import yaml


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    with open(os.path.join(share, "config", "recorded_planning_failure_joint_state.yaml")) as stream:
        joints = yaml.safe_load(stream)["joint_positions"]
    config = MoveItConfigsBuilder(
        "x2_ultra", package_name="agibot_x2_moveit_config"
    ).to_moveit_configs()
    replay = Node(
        package="agibot_x2_manipulation",
        executable="test_post_place_replay",
        output="screen",
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.joint_limits,
            {
                "replay_joint_names": list(joints),
                "replay_joint_positions": [float(value) for value in joints.values()],
                "box_dimensions": [0.15, 0.36, 0.32],
                "table_collision_id": "work_table",
                "velocity_scaling": 0.1,
                "acceleration_scaling": 0.1,
                "motion_planning_mode": "pose_to_pose",
                "planning_log_directory": "/tmp/x2-post-place-replay-traces",
            },
        ],
    )
    return LaunchDescription([
        replay,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {"replay": replay}


class TestReturnReplay(unittest.TestCase):
    def test_return_planner(self, proc_info, replay):
        proc_info.assertWaitForShutdown(process=replay, timeout=180)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit(self, proc_info, replay):
        launch_testing.asserts.assertExitCodes(proc_info, process=replay)
