"""Recover near a detected box without invoking free-space Pregrasp planning."""

import importlib.util
import os
from pathlib import Path
import tempfile

from action_msgs.msg import GoalStatus
from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger
import yaml


spec = importlib.util.spec_from_file_location(
    "cartesian_recovery_fixture", Path(__file__).with_name("test_continue_actions.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    profiles["pick_place_server"] = {"ros__parameters": {
        "simulate_ideal_attachment": True, "execution_position_tolerance": 0.02}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as params:
        yaml.safe_dump(profiles, params)
    return fixture.fixture.make_test_description(params.name, extra_arguments={
        "phase_retry_attempts": "1", "phase_retry_timeout": "40.0",
        "motion_planning_mode": os.environ.get("X2_RECOVERY_TEST_MODE", "pose_to_pose")})


class TestCartesianRecovery(fixture.TestContinueActions):
    def test_continue_all_actions(self):
        rclpy.init()
        self.node = rclpy.create_node("cartesian_recovery_test")
        self.statuses, self.clients = [], []
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.read_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.resume_client = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        validity = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.assertTrue(self.scene.wait_for_service(timeout_sec=40.0))
        self.assertTrue(self.resume_client.wait_for_service(timeout_sec=40.0))
        self.assertTrue(validity.wait_for_service(timeout_sec=40.0))
        publisher = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        state = {"y": 0.0}
        injection = {"phase": "", "future": None}
        physical, logs = [], []

        def publish():
            box = BoxState(instance_id="tag:0", profile_id="small_carton")
            box.header.frame_id = "base_link"
            box.header.stamp = self.node.get_clock().now().to_msg()
            box.pose.pose.position.x, box.pose.pose.position.z = 0.33, 0.14
            box.pose.pose.position.y = state["y"]
            box.pose.pose.orientation.w = 1.0
            publisher.publish(BoxStateArray(boxes=[box]))

        def status(message):
            self.statuses.append(message)
            if (injection["phase"] and message.phase == injection["phase"]
                    and message.status == "running" and injection["future"] is None):
                injection["future"] = self.block(True)

        def operation(name):
            def callback(request, response):
                physical.append(name)
                response.success = True
                return response
            return callback

        def assert_target_contact():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            future = self.read_scene.call_async(request)
            self.wait(future.done, 10.0)
            check = GetStateValidity.Request(group_name="dual_arm",
                                            robot_state=future.result().scene.robot_state)
            future = validity.call_async(check)
            self.wait(future.done, 10.0)
            pairs = [{c.contact_body_1, c.contact_body_2} for c in future.result().contacts]
            touches = {"right_wrist_yaw_link", "right_wrist_pitch_link",
                       "right_wrist_roll_link", "right_hand_pad_link"}
            self.assertTrue(any("grasp_box_tag_0" in pair and pair & touches for pair in pairs),
                            pairs)

        def start_pick(saved):
            goal = Pick.Goal(instance_id="tag:0")
            if saved:
                goal.plan_only = True
                _, result, _ = self.start(Pick, "/pick_box", goal)
                plan = self.succeed(result)
                goal = Pick.Goal(plan_id=plan.plan_id)
            return self.start(Pick, "/pick_box", goal)

        def cleanup():
            injection["phase"] = ""
            _, result, _ = self.start(Place, "/place_box", self.place_goal(Place))
            self.succeed(result)

        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status, qos),
            self.node.create_subscription(Log, "/rosout", lambda msg: logs.append(msg.msg), qos)]
        services = [self.node.create_service(Trigger, "/mujoco_grasp/attach", operation("attach")),
                    self.node.create_service(Trigger, "/mujoco_grasp/detach", operation("release"))]
        timer = self.node.create_timer(0.03, publish)
        try:
            for saved in (False, True):
                # Stop at Approach, then refresh the target into the wrist's
                # current position. A strict Pregrasp start would be invalid.
                state["y"] = 0.0
                injection.update(phase="saved/approach" if saved else "approach", future=None)
                before = len(physical)
                handle, result, task = start_pick(saved)
                self.wait(lambda: self.paused(task) is not None)
                paused = self.paused(task)
                self.assertEqual(paused.phase, injection["phase"])
                injection["phase"] = ""
                marker = len(logs)
                state["y"] = -0.08
                self.resume(task, paused.pause_id)
                self.wait(lambda: self.paused(task, paused.pause_id) is not None)
                paused = self.paused(task, paused.pause_id)
                self.assertEqual(len(physical), before)
                self.set_block(False)
                assert_target_contact()
                self.resume(task, paused.pause_id)
                if os.environ.get("X2_RECOVERY_TEST_MODE") == "closed_chain":
                    # This displaced fixture cannot finish its carry search
                    # within the unchanged budget. Approach must still accept
                    # the wrist contact and pause before attachment, without a
                    # free-space fallback. Cancellation must remain available.
                    self.wait(lambda: self.paused(task, paused.pause_id) is not None)
                    failed = self.paused(task, paused.pause_id)
                    self.assertIn("adaptive carry", failed.failure)
                    self.assertEqual(failed.object_disposition, "not_attached")
                    self.assertEqual(physical, [])
                    self.assertFalse(any("Planning pregrasp candidate" in msg
                                         for msg in logs[marker:]))
                    canceled = handle.cancel_goal_async()
                    self.wait(canceled.done, 10.0)
                    self.assertTrue(canceled.result().goals_canceling)
                    self.wait(result.done, 10.0)
                    self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
                    return
                self.succeed(result)
                self.assertEqual(physical[before:], ["attach"])
                self.assertTrue(any("Cartesian approach recovery" in msg for msg in logs[marker:]))
                self.assertFalse(any("Planning pregrasp candidate" in msg for msg in logs[marker:]))
                cleanup()

            for saved in (False, True):
                # At Attach, small detection changes retain valid contacts;
                # larger changes recover with Cartesian motion only.
                state["y"] = 0.0
                injection.update(phase="saved/attach" if saved else "attach", future=None)
                before = len(physical)
                _, result, task = start_pick(saved)
                self.wait(lambda: self.paused(task) is not None)
                paused = self.paused(task)
                self.assertEqual(paused.phase, injection["phase"])
                injection["phase"] = ""
                marker = len(logs)
                state["y"] = 0.005
                self.resume(task, paused.pause_id)
                self.wait(lambda: self.paused(task, paused.pause_id) is not None)
                paused = self.paused(task, paused.pause_id)
                self.assertEqual(len(physical), before)
                state["y"] = 0.035
                self.set_block(False)
                self.resume(task, paused.pause_id)
                self.succeed(result)
                self.assertEqual(physical[before:], ["attach"])
                self.assertTrue(any("Cartesian approach recovery" in msg for msg in logs[marker:]))
                self.assertFalse(any("Planning pregrasp candidate" in msg for msg in logs[marker:]))
                cleanup()
            self.assertEqual(physical, ["attach", "release"] * 4)
            # A failed Cartesian recovery remains cancelable without dispatching
            # either a fallback Pregrasp or a physical attachment.
            state["y"] = 0.0
            injection.update(phase="approach", future=None)
            handle, result, task = start_pick(False)
            self.wait(lambda: self.paused(task) is not None)
            paused = self.paused(task)
            injection["phase"] = ""
            marker = len(logs)
            state["y"] = -0.08
            self.resume(task, paused.pause_id)
            self.wait(lambda: self.paused(task, paused.pause_id) is not None)
            canceled = handle.cancel_goal_async()
            self.wait(canceled.done, 10.0)
            self.assertTrue(canceled.result().goals_canceling)
            self.wait(result.done, 10.0)
            self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
            self.assertEqual(physical, ["attach", "release"] * 4)
            self.assertFalse(any("Planning pregrasp candidate" in msg for msg in logs[marker:]))
        finally:
            self.node.destroy_timer(timer)
            for service in services:
                self.node.destroy_service(service)
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            for client in self.clients:
                client.destroy()
            for client in (self.scene, self.read_scene, self.resume_client, validity):
                self.node.destroy_client(client)
            self.node.destroy_publisher(publisher)
            self.node.destroy_node()
            rclpy.shutdown()
