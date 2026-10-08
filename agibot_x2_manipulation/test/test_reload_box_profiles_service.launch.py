import os
from copy import deepcopy
import tempfile
import unittest

from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.srv import GetBoxProfiles, ReloadBoxProfiles
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml


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
            "posture_zmq_enabled": "false",
            "use_rviz": "false",
            "use_apriltag": "false",
            "use_dummy_apriltag": "false",
            "box_profiles_file": os.path.join(
                os.path.dirname(__file__), "config", "box_profiles_simulation.yaml"
            ),
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
        cls.temporary = tempfile.TemporaryDirectory(
            prefix="x2-profile-catalog-test-"
        )
        cls.profiles_file = os.path.join(cls.temporary.name, "box_profiles.yaml")
        source = os.path.join(
            os.path.dirname(__file__), "config", "box_profiles_simulation.yaml"
        )
        with open(source) as stream:
            catalog = yaml.safe_load(stream)
        parameters = catalog["/**"]["ros__parameters"]
        template = parameters["box_profiles"]["small_carton"]
        parameters["box_profiles"] = {
            name: deepcopy(template) for name in ("box-a", "box_b")
        }
        parameters["box_profiles"]["box-a"]["tag_ids"] = [190]
        parameters["box_profiles"]["box_b"]["tag_ids"] = [191, 192]
        with open(cls.profiles_file, "w") as stream:
            yaml.safe_dump(catalog, stream)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.temporary.cleanup()

    def catalog(self):
        client = self.node.create_client(GetBoxProfiles, "/get_box_profiles")
        self.assertTrue(client.wait_for_service(timeout_sec=40.0))
        future = client.call_async(GetBoxProfiles.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        response = future.result()
        client.destroy()
        return response

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
        initial = self.catalog()
        self.assertEqual(initial.profile_ids, ["small_carton"])
        self.assertEqual(initial.profile_version, 0)
        validation = self.reload(dry_run=True)
        self.assertTrue(validation.success, validation.message)
        self.assertEqual(validation.profile_version, 0)
        self.assertEqual(
            self.catalog().profile_ids, ["small_carton"],
            "Dry run must retain the active catalog",
        )

        applied = self.reload(dry_run=False)
        self.assertTrue(applied.success, applied.message)
        self.assertEqual(applied.profile_version, 1)
        catalog = self.catalog()
        self.assertEqual(
            catalog.profile_ids, ["box-a", "box_b"],
            "Profiles must be listed without any tag detections",
        )
        self.assertEqual(catalog.profile_version, applied.profile_version)
        self.profiles_file = os.path.join(self.temporary.name, "missing.yaml")
        rejected = self.reload(dry_run=False)
        self.assertFalse(rejected.success)
        self.assertEqual(self.catalog().profile_ids, ["box-a", "box_b"])
        self.assertEqual(self.catalog().profile_version, applied.profile_version)
