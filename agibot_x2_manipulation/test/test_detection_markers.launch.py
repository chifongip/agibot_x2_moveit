"""Verify box markers use fresh, distinct observations and finite lifetimes."""

import time
import unittest

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from agibot_x2_manipulation_msgs.msg import BoxStateArray
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    localizer = Node(
        package="agibot_x2_manipulation", executable="box_localizer_node",
        parameters=[{
            "detections_topic": "/marker_test/detections",
            "box_states_topic": "/marker_test/states",
            "tag_frame": "marker_test_tag", "stable_sample_count": 2,
            "maximum_pose_age": 1.0,
        }],
        remappings=[("/box_markers", "/marker_test/markers")],
    )
    return LaunchDescription([localizer, launch_testing.actions.ReadyToTest()])


class TestDetectionMarkers(unittest.TestCase):
    def test_cached_tf_cannot_create_or_refresh_markers(self):
        rclpy.init()
        node = rclpy.create_node("detection_marker_test")
        markers = []
        states = []
        node.create_subscription(MarkerArray, "/marker_test/markers", markers.append, 10)
        node.create_subscription(BoxStateArray, "/marker_test/states", states.append, 10)
        publisher = node.create_publisher(AprilTagDetectionArray, "/marker_test/detections", 10)
        broadcaster = TransformBroadcaster(node)

        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)

        def transform(stamp):
            message = TransformStamped()
            message.header.frame_id = "base_link"
            message.header.stamp = stamp.to_msg()
            message.child_frame_id = "marker_test_tag"
            message.transform.translation.z = 0.5
            message.transform.rotation.w = 1.0
            broadcaster.sendTransform(message)
            spin(0.05)

        def detection():
            message = AprilTagDetectionArray()
            message.header.stamp = node.get_clock().now().to_msg()
            message.detections = [AprilTagDetection(id=0, decision_margin=30.0)]
            publisher.publish(message)
            spin(0.04)

        try:
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                spin(0.05)
            self.assertGreater(publisher.get_subscription_count(), 0)
            spin(0.3)  # Allow TF discovery before sending the first observation.
            first = node.get_clock().now() - rclpy.duration.Duration(seconds=0.2)
            transform(first)
            for _ in range(3):
                detection()
            self.assertEqual(markers, [])
            self.assertEqual(states, [])
            second = node.get_clock().now() - rclpy.duration.Duration(seconds=0.1)
            transform(second)
            detection()
            self.assertEqual(len(markers), 1)
            marker = markers[0].markers[0]
            lifetime = marker.lifetime.sec + marker.lifetime.nanosec * 1e-9
            self.assertGreater(lifetime, 0)
            self.assertLess(lifetime, 0.95)
            self.assertEqual(marker.header.stamp, second.to_msg())
            for _ in range(3):
                detection()
            self.assertEqual(len(markers), 1)
            self.assertEqual(len(states), 1)
            spin(1.1)
            detection()
            self.assertEqual(len(markers), 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
