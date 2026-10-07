"""Exercise named tables and saved-plan binding with fake joint feedback."""

import importlib.util
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import unittest


spec = importlib.util.spec_from_file_location(
    "table_profiles_fixture", Path(__file__).with_name("test_moved_box_retry.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description(publish_table=False, table_profiles_file=
        Path(__file__).parent / "config" / "table_profiles_simulation.yaml")


class TestTableProfiles(unittest.TestCase):
    def test_named_tables_saved_binding_and_manual_place(self):
        rclpy.init()
        node = rclpy.create_node("table_profiles_workflow_test")
        boxes = node.create_publisher(BoxStateArray, "/box_states", 10)
        tags = node.create_publisher(AprilTagDetectionArray,
            "/front_center_rectify/detections", qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(node)
        scene = node.create_client(GetPlanningScene, "/get_planning_scene")
        clients = []
        feedback_profiles = []
        table_markers = {}
        marker_sub = node.create_subscription(MarkerArray, "/table_markers",
            lambda message: table_markers.update({marker.ns: marker for marker in message.markers
                if marker.action == marker.ADD}),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def publish():
            stamp = node.get_clock().now().to_msg()
            box = BoxState()
            box.header.frame_id = "base_link"
            box.header.stamp = stamp
            box.instance_id = "tag:0"
            box.profile_id = "small_carton"
            box.pose.pose.position.x = 0.33
            box.pose.pose.position.z = 0.14
            box.pose.pose.orientation.w = 1.0
            array = BoxStateArray()
            array.boxes = [box]
            boxes.publish(array)
            detections = AprilTagDetectionArray()
            detections.header.frame_id = "base_link"
            detections.header.stamp = stamp
            for tag_id, x in [(9, 10.0), (10, 12.0)]:
                tf = TransformStamped()
                tf.header.frame_id = "base_link"
                tf.header.stamp = stamp
                tf.child_frame_id = f"tag{tag_id}"
                tf.transform.translation.x = x
                tf.transform.rotation.w = 1.0
                broadcaster.sendTransform(tf)
                detection = AprilTagDetection()
                detection.id = tag_id
                detection.decision_margin = 100.0
                detections.detections.append(detection)
            tags.publish(detections)

        timer = node.create_timer(0.03, publish)

        def wait(future, timeout):
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            self.assertTrue(future.done(), "Timed out waiting for fake workflow")
            return future.result()

        def send(action, name, goal, success=True):
            client = ActionClient(node, action, name)
            clients.append(client)
            self.assertTrue(client.wait_for_server(timeout_sec=40.0))
            handle = wait(client.send_goal_async(goal, feedback_callback=lambda message:
                feedback_profiles.append(message.feedback.table_profile_id)), 10.0)
            self.assertTrue(handle.accepted)
            result = wait(handle.get_result_async(), 100.0)
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED if success
                             else GoalStatus.STATUS_ABORTED, result.result.message)
            return result.result

        try:
            self.assertTrue(scene.wait_for_service(timeout_sec=40.0))
            invalid = send(Pick, "/pick_box", Pick.Goal(
                plan_only=True, instance_id="tag:0", table_profile_id="missing"), False)
            self.assertEqual(invalid.error_code, Pick.Result.INVALID_GOAL)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.03)
            planned = send(Pick, "/pick_box", Pick.Goal(
                plan_only=True, instance_id="tag:0", table_profile_id="second"))
            self.assertEqual(planned.table_profile_id, "second")
            self.assertTrue(planned.plan_id)
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            objects = {obj.id: obj for obj in wait(scene.call_async(request), 5.0)
                       .scene.world.collision_objects}
            self.assertIn("work_table", objects)
            self.assertIn("second_work_table", objects)
            self.assertAlmostEqual(objects["second_work_table"].pose.position.x, 12.1, places=5)
            self.assertEqual(list(objects["second_work_table"].primitives[0].dimensions), [0.8, 0.5, 0.7])
            mismatch = send(Pick, "/pick_box", Pick.Goal(
                plan_id=planned.plan_id, table_profile_id="default"), False)
            self.assertEqual(mismatch.error_code, Pick.Result.INVALID_GOAL)
            replanned = send(Pick, "/pick_box", Pick.Goal(
                plan_only=True, instance_id="tag:0", table_profile_id="second"))
            # Empty selection executes the saved table even though default is tag9.
            picked = send(Pick, "/pick_box", Pick.Goal(plan_id=replanned.plan_id))
            self.assertEqual(picked.table_profile_id, "second")
            self.assertTrue(picked.object_held)
            place = Place.Goal(table_profile_id="second")
            place.place_pose.header.frame_id = "base_link"
            place.place_pose.pose.position.x = 0.35
            place.place_pose.pose.position.z = 0.17
            place.place_pose.pose.orientation.w = 1.0
            place.plan_only = True
            place_plan = send(Place, "/place_box", place)
            placed = send(Place, "/place_box", Place.Goal(
                plan_id=place_plan.plan_id, table_profile_id="second"))
            self.assertEqual(placed.table_profile_id, "second")
            self.assertFalse(placed.object_held)
            self.assertAlmostEqual(placed.achieved_pose.pose.position.x, 0.35, delta=0.05)
            self.assertEqual(set(feedback_profiles), {"second"})
            self.assertIn("detected_table", table_markers)
            self.assertIn("detected_table/second", table_markers)
            self.assertAlmostEqual(table_markers["detected_table/second"].scale.x, 0.8)
        finally:
            for client in clients:
                client.destroy()
            node.destroy_subscription(marker_sub)
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()
