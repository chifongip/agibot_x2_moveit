"""Saved snapshots ignore detections unless selected-target replanning is enabled."""

import importlib.util
from pathlib import Path
import time

from agibot_x2_manipulation_msgs.action import Pick, PickPlace, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, TransformStamped
import launch_testing
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster


def load_fixture(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fixture = load_fixture("saved_snapshot_fixture", "test_moved_box_retry.launch.py")
workflow = load_fixture("saved_snapshot_workflow", "test_dummy_workflow.launch.py")


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description(publish_table=False, table_profiles_file=
        Path(__file__).parent / "config" / "table_profiles_simulation.yaml")


class TestSavedDetectionSnapshot(workflow.TestDummyWorkflow):
    def test_dummy_pick_place_and_pick_place(self):
        boxes = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        tags = self.node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(self.node)
        scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        parameters = self.node.create_client(SetParameters, "/pick_place_server/set_parameters")
        state = {"x": 0.33, "obstacle_x": 2.0, "boxes": True, "tags": {10},
                 "profile": "small_carton", "inject_outage": False, "move_after_pregrasp": False,
                 "toggle_during_execution": False, "clear_preview_blocker": False,
                 "include_obstacle": False}
        statuses = []
        samples = []
        parameter_updates = []
        preview_unblock_requests = []

        def wait(future, timeout=10.0):
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            self.assertTrue(future.done())
            return future.result()

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.03)

        def publish():
            stamp = self.node.get_clock().now().to_msg()
            if state["boxes"]:
                array = BoxStateArray()
                for instance, x, y in [("tag:0", state["x"], 0.0),
                                       ("tag:1", state["obstacle_x"], 1.0)]:
                    if instance == "tag:1" and not state["include_obstacle"]:
                        continue
                    box = BoxState()
                    box.header.frame_id = "base_link"
                    box.header.stamp = stamp
                    box.instance_id, box.profile_id = instance, state["profile"]
                    box.pose.pose.position.x, box.pose.pose.position.y = x, y
                    box.pose.pose.position.z = 0.14
                    box.pose.pose.orientation.w = 1.0
                    array.boxes.append(box)
                boxes.publish(array)
            detections = AprilTagDetectionArray()
            detections.header.frame_id = "base_link"
            detections.header.stamp = stamp
            for tag_id in state["tags"]:
                tf = TransformStamped()
                tf.header.frame_id, tf.header.stamp = "base_link", stamp
                tf.child_frame_id = f"tag{tag_id}"
                tf.transform.translation.x = 10.0 + (tag_id - 9) * 2.0
                tf.transform.rotation.w = 1.0
                broadcaster.sendTransform(tf)
                detection = AprilTagDetection()
                detection.id, detection.decision_margin = tag_id, 100.0
                detections.detections.append(detection)
            tags.publish(detections)

        def status(message):
            statuses.append(message)
            if (state["clear_preview_blocker"] and
                    message.phase == "planning_complete_path" and message.failure):
                state.update(clear_preview_blocker=False, boxes=False, tags=set())
                obj = CollisionObject(id="preview_retry_blocker", operation=CollisionObject.REMOVE)
                request = ApplyPlanningScene.Request()
                request.scene.is_diff = True
                request.scene.robot_state.is_diff = True
                request.scene.world.collision_objects = [obj]
                preview_unblock_requests.append(apply.call_async(request))
            if message.status != "running" or not message.phase.startswith("saved/"):
                return
            if state["toggle_during_execution"]:
                state["toggle_during_execution"] = False
                request = SetParameters.Request(parameters=[
                    Parameter("pick_replan_on_target_movement", value=True).to_parameter_msg()])
                parameter_updates.append(parameters.call_async(request))
            if state["move_after_pregrasp"] and message.phase == "saved/approach":
                state.update(x=0.33, move_after_pregrasp=False)
                publish()
            if state["inject_outage"]:
                state.update(boxes=False, tags=set())
            if message.phase == "saved/carry":
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                samples.append(scene.call_async(request))

        def toggle(enabled):
            request = SetParameters.Request(parameters=[
                Parameter("pick_replan_on_target_movement", value=enabled).to_parameter_msg()])
            result = wait(parameters.call_async(request))
            self.assertTrue(result.results[0].successful)

        def plan(action, topic, instance_id="tag:0"):
            goal = action.Goal(instance_id=instance_id, table_profile_id="second", plan_only=True)
            if action == PickPlace:
                self.place_pose(goal)
            result = self.send_goal(action, topic, goal, 120.0)
            self.assertTrue(result.plan_id)
            return result.plan_id

        def execute(action, topic, plan_id):
            statuses.clear()
            return self.send_goal(action, topic, action.Goal(plan_id=plan_id), 180.0)

        timer = self.node.create_timer(0.03, publish)
        subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status, 50)
        try:
            self.assertTrue(scene.wait_for_service(timeout_sec=40.0))
            self.assertTrue(apply.wait_for_service(timeout_sec=40.0))
            self.assertTrue(parameters.wait_for_service(timeout_sec=40.0))
            spin_for(1.0)
            # Retrying a default-selected target must use the accepted instance,
            # even after the detector stops and an external blocker is removed.
            blocker = CollisionObject(id="preview_retry_blocker", operation=CollisionObject.ADD)
            blocker.header.frame_id = "base_link"
            blocker.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0] * 3)]
            blocker.primitive_poses = [Pose()]
            blocker.primitive_poses[0].orientation.w = 1.0
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.robot_state.is_diff = True
            request.scene.world.collision_objects = [blocker]
            self.assertTrue(wait(apply.call_async(request)).success)
            state["clear_preview_blocker"] = True
            # default table is absent from this saved PickPlace's snapshot.
            plan(PickPlace, "/pick_place", instance_id="")
            self.assertEqual(len(preview_unblock_requests), 1)
            self.assertTrue(wait(preview_unblock_requests[0]).success)
            self.assertTrue(any(s.phase == "planning_complete_path" and s.status == "retrying"
                                for s in statuses))
            state.update(boxes=True, tags={10}, include_obstacle=True)
            spin_for(0.8)
            saved = plan(PickPlace, "/pick_place")
            state.update(x=0.9, obstacle_x=2.2, tags={9, 10}, profile="grey_box",
                         inject_outage=True, toggle_during_execution=True)
            spin_for(0.8)
            # Restore must remove unsaved managed objects and preserve external ones.
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.robot_state.is_diff = True
            for object_id in ["work_table", "grasp_box_tag_99", "external_snapshot_obstacle"]:
                obj = CollisionObject()
                obj.header.frame_id, obj.id = "base_link", object_id
                obj.operation = CollisionObject.ADD
                primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1])
                pose = Pose()
                pose.position.x, pose.orientation.w = 20.0, 1.0
                obj.primitives, obj.primitive_poses = [primitive], [pose]
                request.scene.world.collision_objects.append(obj)
            self.assertTrue(wait(apply.call_async(request)).success)
            result = execute(PickPlace, "/pick_place", saved)
            self.assertIn("without replanning", result.message)
            self.assertFalse(any("refreshed" in s.failure or s.status == "paused" for s in statuses))
            self.assertEqual(len(parameter_updates), 1)
            self.assertTrue(wait(parameter_updates[0]).results[0].successful)
            self.assertTrue(samples)
            objects = {obj.id: obj for obj in wait(samples.pop()).scene.world.collision_objects}
            self.assertNotIn("work_table", objects)
            self.assertNotIn("grasp_box_tag_99", objects)
            self.assertIn("external_snapshot_obstacle", objects)
            self.assertAlmostEqual(objects["grasp_box_tag_1"].pose.position.x, 2.0, places=6)
            # Toggle after planning: execution latches the current parameter.
            state.update(x=0.33, obstacle_x=2.0, boxes=True, tags={10},
                         profile="small_carton", inject_outage=False)
            spin_for(0.8)
            toggle(False)
            saved = plan(Pick, "/pick_box")
            toggle(True)
            state.update(x=0.45, obstacle_x=2.3)
            spin_for(0.3)
            result = execute(Pick, "/pick_box", saved)
            self.assertTrue(result.object_held)
            self.assertIn("after Pick target replanning", result.message)
            self.assertTrue(any(s.phase == "replanning_pick" for s in statuses))
            goal = Place.Goal(table_profile_id="second")
            self.place_pose(goal)
            goal.plan_only = True
            place_plan = self.send_goal(Place, "/place_box", goal, 120.0)
            state.update(boxes=False, tags=set())
            spin_for(0.8)
            placed = execute(Place, "/place_box", place_plan.plan_id)
            self.assertFalse(placed.object_held)
            state.update(boxes=True, tags={10})
            # Complete saved PickPlace must rebuild Place paths for its new grasp.
            state.update(x=0.33, obstacle_x=2.0)
            spin_for(0.3)
            saved = plan(PickPlace, "/pick_place")
            state.update(x=0.45, move_after_pregrasp=True)
            spin_for(0.3)
            result = execute(PickPlace, "/pick_place", saved)
            self.assertIn("after Pick target replanning", result.message)
            self.assertAlmostEqual(result.achieved_pose.pose.position.x, 0.35, delta=0.05)
            self.assertTrue(any(s.phase == "replanning_pick" for s in statuses))
            self.assertFalse(state["move_after_pregrasp"])
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_subscription(subscription)
            self.node.destroy_client(scene)
            self.node.destroy_client(apply)
            self.node.destroy_client(parameters)
            self.node.destroy_publisher(boxes)
            self.node.destroy_publisher(tags)
