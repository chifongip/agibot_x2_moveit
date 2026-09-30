"""Recover moved targets and obstacles using fresh detections, without Continue."""

import os
from pathlib import Path
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
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
    return make_test_description()


def make_test_description(box_profiles_file=None):
    share = get_package_share_directory("agibot_x2_manipulation")
    port = 20000 + os.getpid() % 10000
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "box_pick_place.launch.py")),
        launch_arguments={
            "command_transport": "zmq",
            "zmq_endpoint": f"tcp://*:{port}",
            "posture_zmq_enabled": "false",
            "use_rviz": "false",
            "use_apriltag": "false",
            "use_dummy_apriltag": "false",
            "start_table_tag_detector": "false",
            "perception_3d_source": "none",
            "allow_execution": "true",
            "motion_planning_mode": "pose_to_pose",
            "box_profiles_file": str(box_profiles_file or
                                     Path(__file__).parent / "config" / "box_profiles_simulation.yaml"),
            "manipulation_state_file": f"/tmp/x2_moved_box_retry_{os.getpid()}",
        }.items(),
    )
    fake = Node(
        package="agibot_x2_ros2_control", executable="fake_zmq_joint_states",
        arguments=["--endpoint", f"tcp://127.0.0.1:{port}", "--initial-pose", "locomanipulation"],
        output="screen",
    )
    return LaunchDescription([fake, stack, launch_testing.actions.ReadyToTest()])


class TestMovedBoxRetry(unittest.TestCase):
    def test_replans_moved_target_and_held_scene_without_continue(self):
        rclpy.init()
        node = rclpy.create_node("moved_box_retry_test")
        publisher = node.create_publisher(BoxStateArray, "/box_states", 10)
        positions = {"tag:0": (0.33, 0.0, 0.14), "tag:1": (2.0, 1.0, 0.14)}
        statuses = []
        shifted = False

        def publish():
            message = BoxStateArray()
            for instance, position in positions.items():
                box = BoxState()
                box.header.frame_id = "base_link"
                box.header.stamp = node.get_clock().now().to_msg()
                box.instance_id = instance
                box.profile_id = "small_carton"
                box.pose.pose.position.x, box.pose.pose.position.y, box.pose.pose.position.z = position
                box.pose.pose.orientation.w = 1.0
                message.boxes.append(box)
            publisher.publish(message)

        def status(message):
            nonlocal shifted
            statuses.append(message)
            if not shifted and message.phase == "prepare" and message.status == "running":
                positions["tag:0"] = (0.45, 0.0, 0.14)
                shifted = True
                publish()

        subscription = node.create_subscription(ManipulationTaskStatus, "/manipulation_task_status", status, 10)
        timer = node.create_timer(0.03, publish)

        def wait(future, timeout):
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            self.assertTrue(future.done(), [(s.phase, s.status, s.failure) for s in statuses[-5:]])
            return future.result()

        clients = []
        try:
            pick_client = ActionClient(node, Pick, "/pick_box")
            clients.append(pick_client)
            self.assertTrue(pick_client.wait_for_server(timeout_sec=40.0))
            goal = Pick.Goal()
            goal.instance_id = "tag:0"
            handle = wait(pick_client.send_goal_async(goal), 10.0)
            self.assertTrue(handle.accepted)
            result = wait(handle.get_result_async(), 100.0)
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.message)
            self.assertTrue(shifted)
            self.assertTrue(result.result.object_held)
            self.assertTrue(any("refreshed detections and scene" in s.failure for s in statuses))
            self.assertFalse(any(s.status == "paused" for s in statuses))

            # A moved obstacle while carrying must refresh the scene, preserving
            # the attached box, and automatically retry Place.
            statuses.clear()
            positions["tag:1"] = (2.2, 1.0, 0.14)
            publish()
            # Deliver the new obstacle observation before starting Place.
            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            place_client = ActionClient(node, Place, "/place_box")
            clients.append(place_client)
            self.assertTrue(place_client.wait_for_server(timeout_sec=10.0))
            place = Place.Goal()
            place.place_pose.header.frame_id = "base_link"
            place.place_pose.pose.position.x = 0.35
            place.place_pose.pose.position.z = 0.17
            place.place_pose.pose.orientation.w = 1.0
            handle = wait(place_client.send_goal_async(place), 10.0)
            self.assertTrue(handle.accepted)
            result = wait(handle.get_result_async(), 100.0)
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.message)
            self.assertTrue(any("refreshed detections and scene" in s.failure for s in statuses))
            self.assertFalse(any(s.status == "paused" for s in statuses))
        finally:
            for client in clients:
                client.destroy()
            node.destroy_timer(timer)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
