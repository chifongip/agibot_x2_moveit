"""Exercise adaptive Place's continuation callback with actual X2 kinematics."""

import importlib.util
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_testing
from moveit_configs_utils import MoveItConfigsBuilder
import pytest
import yaml


spec = importlib.util.spec_from_file_location(
    "place_planner_fixture", Path(__file__).with_name("test_dummy_workflow.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    description = fixture.make_test_description()
    config = MoveItConfigsBuilder("x2_ultra", package_name="agibot_x2_moveit_config").to_moveit_configs()
    share = Path(get_package_share_directory("agibot_x2_manipulation"))
    joints = yaml.safe_load((share / "config/recorded_planning_failure_joint_state.yaml").read_text())["joint_positions"]
    replay = Node(package="agibot_x2_manipulation", executable="test_place_preflight_planner",
                  output="screen", parameters=[config.robot_description,
                  config.robot_description_semantic, config.robot_description_kinematics,
                  config.joint_limits, {
                      "replay_joint_names": list(joints),
                      "replay_joint_positions": [float(v) for v in joints.values()],
                      "motion_planning_mode": "pose_to_pose",
                      "lift_height": 0.02,
                      "adaptive_place_position_tolerance": [0.02, 0.0, 0.0],
                      "adaptive_place_yaw_tolerance": 0.0,
                      "carry_search_timeout": 15.0,
                      "return_planning_timeout": 5.0,
                      "minimum_carry_joint_margin": 0.0,
                      "planning_log_directory": "/tmp/x2-place-preflight-planner-traces",
                  }])
    description.add_action(replay)
    return description, {"replay": replay}


class TestPlanner(unittest.TestCase):
    def test_planner(self, proc_info, replay):
        proc_info.assertWaitForShutdown(process=replay, timeout=140)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit(self, proc_info, replay):
        launch_testing.asserts.assertExitCodes(proc_info, process=replay)
