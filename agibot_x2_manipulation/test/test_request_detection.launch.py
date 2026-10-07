"""New Combine planning waits for post-request box and table observations."""

import importlib.util
from pathlib import Path
import time

from agibot_x2_manipulation_msgs.action import PickPlace
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def load_fixture(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fixture = load_fixture("request_detection_fixture", "test_moved_box_retry.launch.py")
workflow = load_fixture("request_detection_workflow", "test_dummy_workflow.launch.py")


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description(publish_table=False)


class TestRequestDetection(workflow.TestDummyWorkflow):
    def test_dummy_pick_place_and_pick_place(self):
        boxes = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        tags = self.node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(self.node)
        client = ActionClient(self.node, PickPlace, "/pick_place")
        scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        cached_stamp = self.node.get_clock().now().to_msg()
        state = {"box_new": False, "table_new": False, "publish": True, "prime": True}

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)

        def publish():
            nonlocal cached_stamp
            if not state["publish"]:
                return
            now = self.node.get_clock().now().to_msg()
            if state["prime"]:
                cached_stamp = now
            target = BoxState(instance_id="tag:0", profile_id="small_carton")
            target.header.frame_id = "base_link"
            target.header.stamp = now if state["box_new"] else cached_stamp
            target.pose.pose.position.x = 0.33 if state["box_new"] else 0.45
            target.pose.pose.position.z = 0.14
            target.pose.pose.orientation.w = 1.0
            obstacle = BoxState(instance_id="tag:1", profile_id="small_carton")
            obstacle.header.frame_id, obstacle.header.stamp = "base_link", cached_stamp
            obstacle.pose.pose.position.x, obstacle.pose.pose.position.y = 2.0, 1.0
            obstacle.pose.pose.orientation.w = 1.0
            boxes.publish(BoxStateArray(boxes=[target, obstacle]))
            stamp = now if state["table_new"] else cached_stamp
            transform = TransformStamped()
            transform.header.frame_id, transform.header.stamp = "base_link", stamp
            transform.child_frame_id = "tag9"
            transform.transform.translation.x = 10.0 if state["table_new"] else 12.0
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            tags.publish(AprilTagDetectionArray(header=transform.header,
                detections=[AprilTagDetection(id=9, decision_margin=100.0)]))

        timer = self.node.create_timer(0.03, publish)
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=40.0))
            self.assertTrue(scene.wait_for_service(timeout_sec=10.0))
            # Seed a fresh but pre-request box/table cache. Keep replaying its old
            # timestamps after the request, so receipt alone cannot unblock it.
            cached_stamp = self.node.get_clock().now().to_msg()
            spin_for(0.3)
            state["prime"] = False
            goal = PickPlace.Goal(instance_id="tag:0", plan_only=True)
            self.place_pose(goal)
            sent = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
            self.assertTrue(sent.done())
            handle = sent.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            spin_for(0.3)
            self.assertFalse(result.done(), "cached box/table unexpectedly satisfied the request")
            state["box_new"] = True
            spin_for(0.3)
            self.assertFalse(result.done(), "pre-request table unexpectedly satisfied the request")
            state["table_new"] = True
            rclpy.spin_until_future_complete(self.node, result, timeout_sec=120.0)
            self.assertTrue(result.done())
            planned = result.result().result
            self.assertTrue(planned.success, planned.message)
            self.assertTrue(planned.plan_id)
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            world = scene.call_async(request)
            rclpy.spin_until_future_complete(self.node, world, timeout_sec=10.0)
            self.assertTrue(world.done())
            objects = {obj.id: obj for obj in world.result().scene.world.collision_objects}
            self.assertAlmostEqual(objects["grasp_box_tag_0"].pose.position.x, 0.33, places=6)
            self.assertIn("work_table", objects)
            self.assertAlmostEqual(objects["work_table"].pose.position.x, 10.0, places=6)
            self.assertNotIn("grasp_box_tag_1", objects)
            # Saved execution deliberately uses the accepted snapshot without
            # requiring another observation after its own execution request.
            state["publish"] = False
            executed = self.send_goal(PickPlace, "/pick_place",
                PickPlace.Goal(plan_id=planned.plan_id), 180.0)
            self.assertIn("without replanning", executed.message)
        finally:
            self.node.destroy_timer(timer)
            client.destroy()
            self.node.destroy_client(scene)
            self.node.destroy_publisher(boxes)
            self.node.destroy_publisher(tags)
