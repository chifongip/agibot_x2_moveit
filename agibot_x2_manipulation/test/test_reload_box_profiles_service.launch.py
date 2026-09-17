import os
import unittest

from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.srv import ReloadBoxProfiles
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    port = 22000 + os.getpid() % 10000
    endpoint = f"tcp://127.0.0.1:{port}"
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "box_pick_place.launch.py")
        ),
        launch_arguments={
            "command_transport": "zmq",
            "zmq_endpoint": f"tcp://*:{port}",
            "use_rviz": "false",
            "use_apriltag": "false",
            "use_dummy_apriltag": "true",
            "start_table_tag_detector": "false",
            "perception_3d_source": "none",
            "allow_execution": "false",
            "motion_planning_mode": "pose_to_pose",
            "manipulation_state_file": f"/tmp/x2_reload_service_{os.getpid()}",
        }.items(),
    )
    feedback = Node(
        package="agibot_x2_ros2_control",
        executable="fake_zmq_joint_states",
        name="reload_service_joint_states",
        output="screen",
        arguments=["--endpoint", endpoint, "--initial-pose", "locomanipulation"],
    )
    return LaunchDescription([feedback, stack, launch_testing.actions.ReadyToTest()])


class TestReloadBoxProfilesService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("reload_box_profiles_service_test")
        cls.profiles_file = os.path.join(
            get_package_share_directory("agibot_x2_manipulation"),
            "config",
            "box_profiles.yaml",
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def reload(self, dry_run):
        client = self.node.create_client(ReloadBoxProfiles, "/reload_box_profiles")
        self.assertTrue(client.wait_for_service(timeout_sec=40.0))
        request = ReloadBoxProfiles.Request()
        request.profiles_file = self.profiles_file
        request.dry_run = dry_run
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        response = future.result()
        client.destroy()
        return response

    def test_validates_and_reloads_both_nodes(self):
        validation = self.reload(dry_run=True)
        self.assertTrue(validation.success, validation.message)
        self.assertEqual(validation.profile_version, 0)

        applied = self.reload(dry_run=False)
        self.assertTrue(applied.success, applied.message)
        self.assertEqual(applied.profile_version, 1)

