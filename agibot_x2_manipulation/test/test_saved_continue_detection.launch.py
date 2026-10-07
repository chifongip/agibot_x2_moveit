"""Continue refreshes saved execution without replaying physical checkpoints."""

import importlib.util
from pathlib import Path
import tempfile

from agibot_x2_manipulation_msgs.action import Pick, PickPlace, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml


def load_fixture(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fixture = load_fixture("saved_continue_fixture", "test_continue_actions.launch.py")


@pytest.mark.launch_test
def generate_test_description():
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    profiles["pick_place_server"] = {"ros__parameters": {
        "simulate_ideal_attachment": True,
        "table_profile_names": ["derived"],
        "table_profiles": {"derived": {
            "tag_id": 10, "tag_frame": "tag10", "collision_id": "derived_table",
            "tabletop_center": [0.0, 0.0, 0.0], "dimensions": [0.01, 0.01, 0.01],
            "place_offset": [0.0, -0.05], "place_yaw": 0.0}}}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as params:
        yaml.safe_dump(profiles, params)
    return fixture.fixture.make_test_description(params.name, extra_arguments={
        "phase_retry_attempts": "1", "phase_retry_timeout": "20.0"})


class TestSavedContinueDetection(fixture.TestContinueActions):
    def test_continue_all_actions(self):
        rclpy.init()
        self.node = rclpy.create_node("saved_continue_detection_test")
        self.statuses, self.clients = [], []
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.read_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.resume_client = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        self.assertTrue(self.scene.wait_for_service(timeout_sec=40.0))
        self.assertTrue(self.resume_client.wait_for_service(timeout_sec=40.0))
        boxes = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        state = {"x": 0.33, "obstacle_x": 2.0, "stamp": None,
                 "derived_table_x": None, "derived_stamp": None, "invalid": False}
        tags = self.node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(self.node)
        injection = {"phase": "", "future": None}
        physical = []

        def publish():
            stamp = state["stamp"] or self.node.get_clock().now().to_msg()
            array = BoxStateArray()
            for instance, x, y in [("tag:0", state["x"], 0.0),
                                   ("tag:1", state["obstacle_x"], 1.0)]:
                box = BoxState(instance_id=instance, profile_id="small_carton")
                box.header.frame_id, box.header.stamp = "base_link", stamp
                box.pose.pose.position.x, box.pose.pose.position.y = x, y
                box.pose.pose.position.z = 0.14
                box.pose.pose.orientation.w = (0.0 if state["invalid"] and
                                               instance == "tag:0" else 1.0)
                array.boxes.append(box)
            boxes.publish(array)
            if state["derived_table_x"] is not None:
                transform = TransformStamped()
                transform.header.frame_id = "base_link"
                transform.header.stamp = (state["derived_stamp"] or
                                          self.node.get_clock().now().to_msg())
                transform.child_frame_id = "tag10"
                transform.transform.translation.x = state["derived_table_x"]
                transform.transform.translation.z = 0.01
                rotation = transform.transform.rotation
                rotation.x, rotation.y, rotation.z, rotation.w = 0.5, -0.5, -0.5, 0.5
                broadcaster.sendTransform(transform)
                tags.publish(AprilTagDetectionArray(header=transform.header,
                    detections=[AprilTagDetection(id=10, decision_margin=100.0)]))

        def status(message):
            self.statuses.append(message)
            if (injection["phase"] and message.phase == injection["phase"]
                    and message.status == "running"
                    and injection["future"] is None):
                injection["future"] = self.block(True)

        def attach(request, response):
            physical.append("attach")
            response.success = True
            return response

        def detach(request, response):
            physical.append("release")
            response.success = True
            return response

        def preview(action_type, topic, goal):
            goal.plan_only = True
            _, result, _ = self.start(action_type, topic, goal)
            planned = self.succeed(result)
            self.assertTrue(planned.plan_id)
            return planned.plan_id

        def continue_after_block(task_id, result, obstacle_x):
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            injection["phase"] = ""
            state["obstacle_x"] = obstacle_x
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            return self.succeed(result)

        def assert_obstacle(x):
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = self.read_scene.call_async(request)
            self.wait(future.done, 10.0)
            objects = {obj.id: obj for obj in future.result().scene.world.collision_objects}
            self.assertAlmostEqual(objects["grasp_box_tag_1"].pose.position.x, x, places=6)

        timer = self.node.create_timer(0.03, publish)
        subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        services = [self.node.create_service(Trigger, "/mujoco_grasp/attach", attach),
                    self.node.create_service(Trigger, "/mujoco_grasp/detach", detach)]
        try:
            # A saved Pick must wait for an observation captured after Continue,
            # even when the automatic target movement toggle remains disabled.
            saved = preview(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
            self.set_block(True)
            _, result, task_id = self.start(Pick, "/pick_box", Pick.Goal(plan_id=saved))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            state.update(x=0.45, obstacle_x=2.2, stamp=self.node.get_clock().now().to_msg())
            publish()
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertIn("after the action request", paused.failure)
            self.assertEqual(physical, [])
            state.update(stamp=None, invalid=True)
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertIn("invalid Continue target observation", paused.failure)
            self.assertEqual(physical, [])
            state["invalid"] = False
            injection.update(phase="saved/attach", future=None)
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            self.assertEqual(self.paused(task_id).phase, "saved/attach")
            self.assertEqual(physical, [])
            completed = continue_after_block(task_id, result, 2.2)
            self.assertIn("Continue replanning", completed.message)
            self.assertEqual(physical, ["attach"])
            assert_obstacle(2.2)

            # Held saved Place can refresh obstacles without requiring or using
            # a new target pose; the detector is deliberately moved elsewhere.
            saved = preview(Place, "/place_box", self.place_goal(Place))
            self.set_block(True)
            state["x"] = 0.9
            _, result, task_id = self.start(Place, "/place_box", Place.Goal(plan_id=saved))
            completed = continue_after_block(task_id, result, 2.4)
            self.assertIn("Continue replanning", completed.message)
            self.assertFalse(completed.object_held)
            self.assertEqual(physical, ["attach", "release"])
            self.assertAlmostEqual(completed.achieved_pose.pose.position.x, 0.35, delta=0.05)
            assert_obstacle(2.4)

            # Pause a combined saved action after release. Rebuild only the
            # remaining retreat/return, preserving the released box geometry.
            state["x"] = 0.33
            goal = self.place_goal(PickPlace)
            goal.instance_id = "tag:0"
            saved = preview(PickPlace, "/pick_place", goal)
            injection.update(phase="saved/retreat", future=None)
            _, result, task_id = self.start(PickPlace, "/pick_place", PickPlace.Goal(plan_id=saved))
            self.wait(lambda: self.paused(task_id) is not None)
            self.assertEqual(self.paused(task_id).object_disposition, "released")
            state["x"] = 0.9
            completed = continue_after_block(task_id, result, 2.6)
            self.assertIn("Continue replanning", completed.message)
            self.assertTrue(any(s.task_id == task_id and s.phase == "saved/prepare_direct"
                                for s in self.statuses))
            self.assertEqual(physical, ["attach", "release", "attach", "release"])
            self.assertAlmostEqual(completed.achieved_pose.pose.position.x, 0.35, delta=0.05)
            assert_obstacle(2.6)

            # A saved table-derived Place must retain provenance and recompute
            # its target from the post-Continue table, with attachment unchanged.
            state["x"] = 0.33
            saved = preview(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
            _, result, _ = self.start(Pick, "/pick_box", Pick.Goal(plan_id=saved))
            self.succeed(result)
            state["derived_table_x"] = 0.30
            saved = preview(Place, "/place_box", Place.Goal(table_profile_id="derived"))
            self.set_block(True)
            _, result, task_id = self.start(Place, "/place_box", Place.Goal(plan_id=saved))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            state.update(derived_table_x=0.34, derived_stamp=self.node.get_clock().now().to_msg())
            publish()
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertIn("table tag pose captured after the action request", paused.failure)
            self.assertEqual(physical, ["attach", "release"] * 2 + ["attach"])
            state.update(derived_stamp=None, obstacle_x=2.8)
            self.resume(task_id, paused.pause_id)
            completed = self.succeed(result)
            self.assertIn("Continue replanning", completed.message)
            self.assertAlmostEqual(completed.achieved_pose.pose.position.x, 0.39, delta=0.015)
            self.assertEqual(physical, ["attach", "release"] * 3)
        finally:
            for service in services:
                self.node.destroy_service(service)
            for client in self.clients:
                client.destroy()
            self.node.destroy_timer(timer)
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(boxes)
            self.node.destroy_publisher(tags)
            self.node.destroy_client(self.scene)
            self.node.destroy_client(self.read_scene)
            self.node.destroy_client(self.resume_client)
            self.node.destroy_node()
            rclpy.shutdown()
