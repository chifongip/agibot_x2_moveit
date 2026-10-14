"""Require a valid exit before ordinary or saved Place dispatches release."""

import importlib.util
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_FeedbackMessage
import launch_testing
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
import yaml


spec = importlib.util.spec_from_file_location(
    "place_preflight_fixture", Path(__file__).with_name("test_continue_actions.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
trace = Path(f"/tmp/x2_place_preflight_{os.getpid()}.jsonl")


@pytest.mark.launch_test
def generate_test_description():
    return make_test_description(True)


def make_test_description(preflight_enabled):
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    profiles["pick_place_server"] = {"ros__parameters": {"simulate_ideal_attachment": True}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as params:
        yaml.safe_dump(profiles, params)
    return fixture.fixture.make_test_description(params.name, extra_arguments={
        "phase_retry_attempts": "1", "phase_retry_timeout": "20.0",
        "place_release_preflight_enabled": str(preflight_enabled).lower(),
        "planning_log_file": str(trace)})


class TestPlacePreflight(fixture.TestContinueActions):
    preflight_enabled = True

    def test_continue_all_actions(self):
        rclpy.init()
        self.node = rclpy.create_node("place_preflight_test")
        self.statuses, self.clients = [], []
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.resume_client = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        read_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        fk = self.node.create_client(GetPositionFK, "/compute_fk")
        parameters = self.node.create_client(GetParameters, "/pick_place_server/get_parameters")
        for client in (self.scene, self.resume_client, read_scene, fk, parameters):
            self.assertTrue(client.wait_for_service(timeout_sec=40.0))
        publisher = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        joints, physical = {}, []
        active_task = [None]
        injection = [None]
        ready_pose = [None]
        checkpoint_probe = [False]

        def publish():
            box = BoxState(instance_id="tag:0", profile_id="small_carton")
            box.header.frame_id = "base_link"
            box.header.stamp = self.node.get_clock().now().to_msg()
            box.pose.pose.position.x, box.pose.pose.position.z = 0.33, 0.14
            box.pose.pose.orientation.w = 1.0
            publisher.publish(BoxStateArray(boxes=[box]))

        def attach(request, response):
            physical.append("attach")
            response.success = True
            return response

        def release(request, response):
            physical.append("release")
            response.success = True
            return response

        def blocker(present):
            obj = CollisionObject(id="pre_release_ready_blocker")
            obj.header.frame_id = "base_link"
            obj.operation = obj.ADD if present else obj.REMOVE
            if present:
                obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])]
                obj.primitive_poses = [ready_pose[0]]
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = request.scene.robot_state.is_diff = True
            request.scene.world.collision_objects = [obj]
            return self.scene.call_async(request)

        def controller_feedback(message):
            # Controller feedback proves the placement motion has begun, after
            # its initial complete-path preflight. Block only the later Ready pose.
            latest = next((s for s in reversed(self.statuses)
                           if s.task_id == active_task[0]), None)
            if (latest is not None and latest.phase in ("place", "saved/place")
                    and not checkpoint_probe[0] and injection[0] is None
                    and ready_pose[0] is not None):
                injection[0] = blocker(True)

        def status(message):
            self.statuses.append(message)
            if (checkpoint_probe[0] and message.task_id == active_task[0]
                    and message.phase == "release" and message.status == "running"
                    and injection[0] is None):
                # The completed placement is stationary while the server waits
                # for settled feedback. Put an external obstacle at that hand.
                request = GetPositionFK.Request()
                request.header.frame_id = "base_link"
                request.fk_link_names = ["left_hand_pad_link"]
                request.robot_state.joint_state.name = list(joints)
                request.robot_state.joint_state.position = list(joints.values())
                injection[0] = fk.call_async(request)

                def add_obstacle(future):
                    self.assertEqual(future.result().error_code.val, 1)
                    ready_pose[0] = future.result().pose_stamped[0].pose
                    injection[0] = blocker(True)

                injection[0].add_done_callback(add_obstacle)

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            self.node.create_subscription(ManipulationTaskStatus, "/manipulation_task_status",
                                          status, qos),
            self.node.create_subscription(JointState, "/joint_states",
                                          lambda m: joints.update(zip(m.name, m.position)), 10),
            self.node.create_subscription(FollowJointTrajectory_FeedbackMessage,
                                          "/dual_arm_controller/follow_joint_trajectory/_action/feedback",
                                          controller_feedback, 10),
        ]
        services = [self.node.create_service(Trigger, "/mujoco_grasp/attach", attach),
                    self.node.create_service(Trigger, "/mujoco_grasp/detach", release)]
        timer = self.node.create_timer(0.03, publish)
        try:
            future = parameters.call_async(GetParameters.Request(
                names=["place_release_preflight_enabled"]))
            self.wait(future.done, 10.0)
            self.assertEqual(future.result().values[0].bool_value, self.preflight_enabled)
            self.wait(lambda: bool(joints))
            srdf = Path(get_package_share_directory("agibot_x2_moveit_config")) / "config/x2_ultra.srdf"
            ready = ET.parse(srdf).find(".//group_state[@name='ready'][@group='dual_arm']")
            configuration = {**joints, **{j.attrib["name"]: float(j.attrib["value"]) for j in ready}}
            request = GetPositionFK.Request()
            request.header.frame_id = "base_link"
            request.fk_link_names = ["right_hand_pad_link"]
            request.robot_state.joint_state.name = list(configuration)
            request.robot_state.joint_state.position = list(configuration.values())
            future = fk.call_async(request)
            self.wait(future.done, 10.0)
            self.assertEqual(future.result().error_code.val, 1)
            ready_pose[0] = future.result().pose_stamped[0].pose

            for saved in (False, True):
                _, result, _ = self.start(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
                self.succeed(result)
                goal = self.place_goal(Place)
                if saved:
                    goal.plan_only = True
                    _, result, _ = self.start(Place, "/place_box", goal)
                    preview = self.succeed(result)
                    goal = Place.Goal(plan_id=preview.plan_id)
                before_searches = sum(r["event"] == "pose_search_summary" for r in
                                      map(json.loads, trace.read_text().splitlines()))
                injection[0] = None
                if not saved:
                    # A blocked complete return must reject all placements
                    # before executing even the initial placement trajectory.
                    initial_joints = dict(joints)
                    injection[0] = blocker(True)
                    self.wait(injection[0].done, 10.0)
                    self.assertTrue(injection[0].result().success)
                _, result, task = self.start(Place, "/place_box", goal)
                active_task[0] = task
                if not saved:
                    self.wait(lambda: self.paused(task) is not None)
                    paused = self.paused(task)
                    self.assertEqual(paused.phase, "place")
                    self.assertEqual(physical, ["attach"])
                    for name, value in initial_joints.items():
                        self.assertAlmostEqual(joints[name], value, places=6)
                    future = blocker(False)
                    self.wait(future.done, 10.0)
                    self.assertTrue(future.result().success)
                    injection[0] = None
                    self.resume(task, paused.pause_id)
                    previous_pause = paused.pause_id
                else:
                    previous_pause = 0
                self.wait(lambda: injection[0] is not None and injection[0].done())
                self.assertTrue(injection[0].result().success)
                self.wait(lambda: self.paused(task, previous_pause) is not None)
                paused = self.paused(task, previous_pause)
                if self.preflight_enabled:
                    self.assertEqual(paused.phase, "saved/release" if saved else "release")
                elif saved:
                    self.assertTrue(paused.phase.startswith("saved/ready"), paused.phase)
                else:
                    self.assertEqual(paused.phase, "from_prepare_to_ready")
                self.assertIn("pre_release_ready_blocker", paused.failure)
                if self.preflight_enabled:
                    self.assertNotEqual(paused.object_disposition, "released")
                    self.assertEqual(physical, ["attach", "release"] * int(saved) + ["attach"])
                else:
                    self.assertEqual(paused.object_disposition, "released")
                    self.assertEqual(physical, ["attach", "release"] * (int(saved) + 1))
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                future = read_scene.call_async(request)
                self.wait(future.done, 10.0)
                self.assertEqual(bool(future.result().scene.robot_state.attached_collision_objects),
                                 self.preflight_enabled)
                if saved:
                    searches = sum(r["event"] == "pose_search_summary" for r in
                                   map(json.loads, trace.read_text().splitlines()))
                    self.assertEqual(searches, before_searches, "saved release performed new planning")
                future = blocker(False)
                self.wait(future.done, 10.0)
                self.assertTrue(future.result().success)
                self.resume(task, paused.pause_id)
                self.succeed(result)
                self.assertEqual(physical, ["attach", "release"] * (int(saved) + 1))
                active_task[0] = None

            if not self.preflight_enabled:
                _, result, _ = self.start(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
                self.succeed(result)
                checkpoint_probe[0], injection[0] = True, None
                _, result, task = self.start(Place, "/place_box", self.place_goal(Place))
                active_task[0] = task
                self.wait(lambda: self.paused(task) is not None)
                paused = self.paused(task)
                self.assertEqual(paused.phase, "release")
                self.assertIn("collision", paused.failure)
                self.assertIn("pre_release_ready_blocker", paused.failure)
                self.assertEqual(physical, ["attach", "release"] * 2 + ["attach"])
                self.assertNotEqual(paused.object_disposition, "released")
                before_resume_searches = sum(r["event"] == "pose_search_summary" for r in
                                            map(json.loads, trace.read_text().splitlines()))
                checkpoint_probe[0] = False
                future = blocker(False)
                self.wait(future.done, 10.0)
                self.assertTrue(future.result().success)
                self.resume(task, paused.pause_id)
                self.succeed(result)
                self.assertEqual(physical, ["attach", "release"] * 3)
                searches = sum(r["event"] == "pose_search_summary" for r in
                               map(json.loads, trace.read_text().splitlines()))
                self.assertEqual(searches, before_resume_searches,
                                 "collision recovery performed unnecessary placement correction")
        finally:
            self.node.destroy_timer(timer)
            for service in services:
                self.node.destroy_service(service)
            for sub in subscriptions:
                self.node.destroy_subscription(sub)
            for client in self.clients:
                client.destroy()
            for client in (self.scene, self.resume_client, read_scene, fk, parameters):
                self.node.destroy_client(client)
            self.node.destroy_publisher(publisher)
            self.node.destroy_node()
            rclpy.shutdown()
