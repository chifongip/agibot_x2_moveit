import os
import tempfile
import time
import unittest

from agibot_x2_manipulation_msgs.srv import ReloadBoxProfiles
from agibot_x2_manipulation_msgs.msg import BoxStateArray
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    localizer = Node(
        package="agibot_x2_manipulation",
        executable="box_localizer_node",
        output="screen",
    )
    return LaunchDescription([localizer, launch_testing.actions.ReadyToTest()])


class TestBoxProfileReload(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("box_profile_reload_test")
        cls.catalog = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        cls.catalog.write(
            """/**:
  ros__parameters:
    box_profiles_tag_frame_prefix: tag
    box_profiles:
      bottom_container:
        docking_profile_ids: [box_b_front, box_b_side]
        default_docking_profile: box_b_side
        tag_ids: [42]
        dimensions: [0.2, 0.3, 0.3]
        tag_to_box_center_pose: [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0]
        pregrasp_distance: 0.08
        contact_height_offset: 0.0
        carry_pose_a: [0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
"""
        )
        cls.catalog.close()

    @classmethod
    def tearDownClass(cls):
        os.unlink(cls.catalog.name)
        cls.node.destroy_node()
        rclpy.shutdown()

    def reload(self, dry_run):
        client = self.node.create_client(
            ReloadBoxProfiles, "/box_localizer/reload_box_profiles"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = ReloadBoxProfiles.Request()
        request.profiles_file = self.catalog.name
        request.dry_run = dry_run
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        response = future.result()
        client.destroy()
        return response

    def test_validates_then_reloads_catalog(self):
        dry_run = self.reload(dry_run=True)
        self.assertTrue(dry_run.success, dry_run.message)
        self.assertEqual(dry_run.profile_version, 0)

        applied = self.reload(dry_run=False)
        self.assertTrue(applied.success, applied.message)
        self.assertEqual(applied.profile_version, 1)


        observations = []
        subscription = self.node.create_subscription(
            BoxStateArray, "/box_states", observations.append, 10)
        detections = self.node.create_publisher(AprilTagDetectionArray, "/detections", 10)
        broadcaster = TransformBroadcaster(self.node)
        deadline = time.monotonic() + 5.0
        while not observations and time.monotonic() < deadline:
            transform = TransformStamped()
            transform.header.frame_id = "base_link"
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.child_frame_id = "tag42"
            transform.transform.translation.x = 0.4
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            detections.publish(AprilTagDetectionArray(
                header=transform.header,
                detections=[AprilTagDetection(id=42, decision_margin=100.0)]))
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(observations, "Reloaded box must publish docking associations")
        box = observations[-1].boxes[0]
        self.assertEqual(box.docking_profile_ids, ["box_b_front", "box_b_side"])
        self.assertEqual(box.default_docking_profile, "box_b_side")
        self.assertEqual(box.tag_frame, "tag42")
        self.node.destroy_subscription(subscription)
