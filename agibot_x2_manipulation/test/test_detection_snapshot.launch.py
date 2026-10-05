"""Keep accepted collision geometry while live detections change between phases."""

import importlib.util
from pathlib import Path
import time

from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
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


moved = load_fixture("snapshot_moved_fixture", "test_moved_box_retry.launch.py")
workflow = load_fixture("snapshot_workflow_fixture", "test_dummy_workflow.launch.py")


@pytest.mark.launch_test
def generate_test_description():
    return moved.make_test_description(publish_table=False)


class TestDetectionSnapshot(workflow.TestDummyWorkflow):
    def test_dummy_pick_place_and_pick_place(self):
        boxes = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        tags = self.node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(self.node)
        scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.assertTrue(scene.wait_for_service(timeout_sec=40.0))
        state = {"table_x": 10.0, "obstacle_x": 2.0, "target_x": 0.33,
                 "publish_table": True}
        samples = {}
        shifted = set()

        def publish():
            stamp = self.node.get_clock().now().to_msg()
            message = BoxStateArray()
            for instance, x, y in [("tag:0", state["target_x"], 0.0),
                                   ("tag:1", state["obstacle_x"], 1.0)]:
                box = BoxState()
                box.header.frame_id = "base_link"
                box.header.stamp = stamp
                box.instance_id = instance
                box.profile_id = "small_carton"
                box.pose.pose.position.x = x
                box.pose.pose.position.y = y
                box.pose.pose.position.z = 0.14
                box.pose.pose.orientation.w = 1.0
                message.boxes.append(box)
            boxes.publish(message)
            if state["publish_table"]:
                transform = TransformStamped()
                transform.header.frame_id = "base_link"
                transform.header.stamp = stamp
                transform.child_frame_id = "tag9"
                transform.transform.translation.x = state["table_x"]
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                detection = AprilTagDetection()
                detection.id = 9
                detection.family = "tag36h11"
                detection.decision_margin = 100.0
                array = AprilTagDetectionArray()
                array.header.frame_id = "base_link"
                array.header.stamp = stamp
                array.detections = [detection]
                tags.publish(array)

        def status(message):
            if message.status != "running":
                return
            # These shifts pass freshness/movement checks. They must not rewrite
            # the collision scene at later planning or execution checkpoints.
            key = (message.action, message.phase)
            if key == ("pick", "prepare") and key not in shifted:
                state.update(table_x=10.002, obstacle_x=2.004, target_x=0.332)
                shifted.add(key)
            elif key == ("place", "perception") and key not in shifted:
                state.update(table_x=10.004, obstacle_x=2.008)
                shifted.add(key)
            if key in [("pick", "carry"), ("place", "to_prepare")] and key not in samples:
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                samples[key] = scene.call_async(request)
            # A table detector outage after release must not require a new table
            # observation for retreat/Prepare/Ready with an accepted snapshot.
            if key == ("place", "release_scene"):
                state["publish_table"] = False

        subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status, 50)
        timer = self.node.create_timer(0.03, publish)
        try:
            # Pick no longer waits for a table. Publish stable observations
            # after the server is ready before testing snapshot retention.
            ready = ActionClient(self.node, Pick, "/pick_box")
            try:
                self.assertTrue(ready.wait_for_server(timeout_sec=40.0))
            finally:
                ready.destroy()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.03)
            picked = self.send_goal(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"), 120.0)
            self.assertTrue(picked.object_held)
            # Allow the changed stable table observation to reach the tracker
            # before Place captures its own, independent snapshot.
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.03)
            place = Place.Goal()
            self.place_pose(place)
            self.send_goal(Place, "/place_box", place, 120.0)
            self.assertEqual(len(shifted), 2)
            for key, table_x, obstacle_x in [
                    (("pick", "carry"), 10.0, 2.0),
                    (("place", "to_prepare"), 10.002, 2.004)]:
                self.assertIn(key, samples)
                future = samples[key]
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                self.assertTrue(future.done())
                objects = {obj.id: obj for obj in future.result().scene.world.collision_objects}
                # MoveIt serializes these single-shape boxes with their world
                # transform in CollisionObject.pose and a local primitive pose.
                self.assertAlmostEqual(
                    objects["work_table"].pose.position.x, table_x, places=6)
                self.assertAlmostEqual(
                    objects["grasp_box_tag_1"].pose.position.x, obstacle_x, places=6)
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_subscription(subscription)
            self.node.destroy_client(scene)
            self.node.destroy_publisher(boxes)
            self.node.destroy_publisher(tags)
