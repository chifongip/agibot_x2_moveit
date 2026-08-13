import os
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick, PickPlace, Place
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    port = 20000 + os.getpid() % 10000
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
            "perception_3d_source": "none",
            "allow_execution": "true",
            "motion_planning_mode": "pose_to_pose",
            "manipulation_state_file": f"/tmp/x2_dummy_workflow_{os.getpid()}",
        }.items(),
    )
    feedback = Node(
        package="agibot_x2_ros2_control",
        executable="fake_zmq_joint_states",
        name="dummy_x2_joint_states",
        output="screen",
        arguments=["--endpoint", endpoint, "--initial-pose", "locomanipulation"],
    )
    return LaunchDescription([feedback, stack, launch_testing.actions.ReadyToTest()])


class TestDummyWorkflow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dummy_workflow_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_goal(self, action_type, name, goal, timeout):
        client = ActionClient(self.node, action_type, name)
        self.assertTrue(client.wait_for_server(timeout_sec=40.0))
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        self.assertTrue(send_future.done())
        handle = send_future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
        self.assertTrue(result_future.done())
        wrapped = result_future.result()
        self.assertEqual(
            wrapped.status,
            GoalStatus.STATUS_SUCCEEDED,
            wrapped.result.message if wrapped.result else "action returned no result",
        )
        self.assertTrue(wrapped.result.success, wrapped.result.message)
        client.destroy()
        return wrapped.result

    @staticmethod
    def place_pose(goal):
        goal.place_pose.header.frame_id = "base_link"
        goal.place_pose.pose.position.x = 0.35
        goal.place_pose.pose.position.z = 0.17
        goal.place_pose.pose.orientation.w = 1.0

    def test_dummy_pick_place_and_pick_place(self):
        pick_goal = Pick.Goal()
        pick_goal.plan_only = True
        pick_result = self.send_goal(Pick, "/pick_box", pick_goal, 45.0)
        self.assertFalse(pick_result.object_held)

        pick_place_goal = PickPlace.Goal()
        self.place_pose(pick_place_goal)
        pick_place_goal.plan_only = True
        self.send_goal(PickPlace, "/pick_place", pick_place_goal, 45.0)

        pick_goal.plan_only = False
        pick_result = self.send_goal(Pick, "/pick_box", pick_goal, 70.0)
        self.assertTrue(pick_result.object_held)

        place_goal = Place.Goal()
        self.place_pose(place_goal)
        place_goal.plan_only = False
        place_result = self.send_goal(Place, "/place_box", place_goal, 70.0)
        self.assertFalse(place_result.object_held)

        pick_place_goal.plan_only = False
        self.send_goal(PickPlace, "/pick_place", pick_place_goal, 100.0)
