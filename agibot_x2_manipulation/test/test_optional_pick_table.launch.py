"""Standalone Pick accepts absent tables but still requires a detected box."""

import importlib.util
from pathlib import Path
import time
import unittest

from action_msgs.msg import GoalStatus
from agibot_x2_manipulation_msgs.action import Pick, PickPlace, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, TransformStamped
import launch_testing
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
import pytest
import rclpy
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster


spec = importlib.util.spec_from_file_location(
    "optional_table_fixture", Path(__file__).with_name("test_moved_box_retry.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description(publish_table=False)


class TestOptionalPickTable(unittest.TestCase):
    def test_optional_table_pick_and_required_tag_place(self):
        rclpy.init()
        node = rclpy.create_node("optional_pick_table_test")
        boxes = node.create_publisher(BoxStateArray, "/box_states", 10)
        tables = node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", 10)
        broadcaster = TransformBroadcaster(node)
        scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        state = {"boxes": True, "table": False}
        clients = []

        def publish():
            stamp = node.get_clock().now().to_msg()
            if state["boxes"]:
                box = BoxState()
                box.header.frame_id = "base_link"
                box.header.stamp = stamp
                box.instance_id = "tag:0"
                box.profile_id = "small_carton"
                box.pose.pose.position.x = 0.33
                box.pose.pose.position.z = 0.14
                box.pose.pose.orientation.w = 1.0
                boxes.publish(BoxStateArray(boxes=[box]))
            if state["table"]:
                transform = TransformStamped()
                transform.header.frame_id = "base_link"
                transform.header.stamp = stamp
                transform.child_frame_id = "tag9"
                transform.transform.translation.x = 10.0
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                tables.publish(AprilTagDetectionArray(
                    header=transform.header,
                    detections=[AprilTagDetection(id=9, decision_margin=100.0)]))

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        def wait(future, timeout=10):
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            self.assertTrue(future.done(), "ROS response timeout")
            return future.result()

        def action(kind, topic, goal, success=True):
            client = ActionClient(node, kind, topic)
            clients.append(client)
            self.assertTrue(client.wait_for_server(timeout_sec=40))
            handle = wait(client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            wrapped = wait(handle.get_result_async(), 90)
            self.assertEqual(wrapped.status,
                             GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED,
                             wrapped.result.message)
            self.assertEqual(wrapped.result.success, success, wrapped.result.message)
            return wrapped.result

        def world():
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            objects = wait(scene_client.call_async(req)).scene.world.collision_objects
            return [obj.id for obj in objects]

        def place(with_table=True):
            state["table"] = with_table
            spin(0.5)
            goal = Place.Goal()
            goal.place_pose.header.frame_id = "base_link"
            goal.place_pose.pose.position.x = 0.35
            goal.place_pose.pose.position.z = 0.17
            goal.place_pose.pose.orientation.w = 1.0
            action(Place, "/place_box", goal)

        timer = node.create_timer(0.03, publish)
        try:
            self.assertTrue(scene_client.wait_for_service(timeout_sec=40))
            self.assertTrue(apply_client.wait_for_service(timeout_sec=10))
            # A previous table must not survive an absent optional observation.
            stale = CollisionObject()
            stale.header.frame_id = "base_link"
            stale.id = "work_table"
            stale.operation = CollisionObject.ADD
            stale.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                              dimensions=[0.1, 0.1, 0.1])]
            pose = Pose()
            pose.position.x = 10.0
            pose.orientation.w = 1.0
            stale.primitive_poses = [pose]
            req = ApplyPlanningScene.Request()
            req.scene.is_diff = True
            req.scene.world.collision_objects = [stale]
            self.assertTrue(wait(apply_client.call_async(req)).success)
            manual = PickPlace.Goal(instance_id="tag:0", plan_only=True)
            manual.place_pose.header.frame_id = "base_link"
            manual.place_pose.pose.position.x = 0.35
            manual.place_pose.pose.position.z = 0.17
            manual.place_pose.pose.orientation.w = 1.0
            planned = action(PickPlace, "/pick_place", manual)
            self.assertTrue(planned.plan_id)
            preview = action(Pick, "/pick_box", Pick.Goal(instance_id="tag:0", plan_only=True))
            self.assertTrue(preview.plan_id)
            self.assertNotIn("work_table", world())
            action(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
            # Empty Place target genuinely needs tag9 and must still fail without it.
            failed = action(Place, "/place_box", Place.Goal(plan_only=True), success=False)
            self.assertIn("table tag", failed.message)
            self.assertTrue(failed.object_held)
            place(with_table=False)
            self.assertNotIn("work_table", world())
            state["table"] = True
            spin(0.5)

            # Fresh table is modeled; losing it after preview must not block saved Pick.
            preview = action(Pick, "/pick_box", Pick.Goal(instance_id="tag:0", plan_only=True))
            self.assertIn("work_table", world())
            state["table"] = False
            spin(3.6)
            action(Pick, "/pick_box", Pick.Goal(instance_id="tag:0", plan_id=preview.plan_id))
            place()

            state["boxes"] = False
            spin(3.6)
            failed = action(
                Pick, "/pick_box", Pick.Goal(instance_id="tag:0", plan_only=True),
                success=False)
            self.assertIn("box", failed.message)
        finally:
            node.destroy_timer(timer)
            for client in clients:
                client.destroy()
            node.destroy_node()
            rclpy.shutdown()
