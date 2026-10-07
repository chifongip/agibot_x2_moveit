"""Exercise Continue before and after physical checkpoints for every task action."""

import importlib.util
from pathlib import Path
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import MoveCarryPose, Pick, PickPlace, Place, ResetManipulation
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
from geometry_msgs.msg import Pose, TransformStamped
import launch_testing
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml

_spec = importlib.util.spec_from_file_location(
    "continue_actions_fixture", Path(__file__).with_name("test_moved_box_retry.launch.py"))
fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    profiles["pick_place_server"] = {
        "ros__parameters": {"simulate_ideal_attachment": True}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as params:
        yaml.safe_dump(profiles, params)
    return fixture.make_test_description(params.name, extra_arguments={
        "phase_retry_attempts": "1", "phase_retry_timeout": "20.0"})


class TestContinueActions(unittest.TestCase):
    def wait(self, predicate, timeout=100.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertTrue(predicate(), [(s.action, s.phase, s.status, s.failure)
                                      for s in self.statuses[-8:]])

    def block(self, present):
        obj = CollisionObject()
        obj.id = "continue_actions_blocker"
        obj.header.frame_id = "base_link"
        obj.operation = CollisionObject.ADD if present else CollisionObject.REMOVE
        if present:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [4.0, 4.0, 4.0]
            obj.primitives = [primitive]
            pose = Pose()
            pose.orientation.w = 1.0
            obj.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [obj]
        return self.scene.call_async(request)

    def set_block(self, present):
        future = self.block(present)
        self.wait(future.done, 10.0)
        self.assertTrue(future.result().success)

    def start(self, action_type, topic, goal):
        client = ActionClient(self.node, action_type, topic)
        self.clients.append(client)
        self.assertTrue(client.wait_for_server(timeout_sec=40.0))
        sent = client.send_goal_async(goal)
        self.wait(sent.done, 10.0)
        handle = sent.result()
        self.assertTrue(handle.accepted)
        return handle, handle.get_result_async(), bytes(handle.goal_id.uuid).hex()

    def paused(self, task_id, after_pause=0):
        return next((s for s in reversed(self.statuses) if s.task_id == task_id
                     and s.status == "paused" and s.pause_id > after_pause), None)

    def resume(self, task_id, pause_id):
        request = ContinueManipulation.Request(task_id=task_id, pause_id=pause_id)
        result = self.resume_client.call_async(request)
        self.wait(result.done, 10.0)
        self.assertTrue(result.result().success, result.result().message)

    def succeed(self, result):
        self.wait(result.done)
        wrapped = result.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED, wrapped.result.message)
        self.assertTrue(wrapped.result.success, wrapped.result.message)
        return wrapped.result

    @staticmethod
    def place_goal(action_type):
        goal = action_type.Goal()
        goal.place_pose.header.frame_id = "base_link"
        goal.place_pose.pose.position.x = 0.35
        goal.place_pose.pose.position.z = 0.17
        goal.place_pose.pose.orientation.w = 1.0
        return goal

    def test_continue_all_actions(self):
        rclpy.init()
        self.node = rclpy.create_node("continue_actions_test")
        self.statuses, self.clients = [], []
        joints = {}
        position = [0.33, 0.0, 0.14]
        obstacle_x = [2.0]
        frozen_stamp = [None]
        place_frame_x = [0.0]
        broadcaster = TransformBroadcaster(self.node)
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.read_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.resume_client = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        self.assertTrue(self.scene.wait_for_service(timeout_sec=40.0))
        self.assertTrue(self.resume_client.wait_for_service(timeout_sec=40.0))
        publisher = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        injection = {"action": "", "phase": "", "future": None}

        def publish():
            transform = TransformStamped()
            transform.header.frame_id = "base_link"
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.child_frame_id = "continue_place_frame"
            transform.transform.translation.x = place_frame_x[0]
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            box = BoxState()
            box.header.frame_id = "base_link"
            box.header.stamp = frozen_stamp[0] or self.node.get_clock().now().to_msg()
            box.instance_id, box.profile_id = "tag:0", "small_carton"
            box.pose.pose.position.x, box.pose.pose.position.y, box.pose.pose.position.z = position
            box.pose.pose.orientation.w = 1.0
            obstacle = BoxState()
            obstacle.header = box.header
            obstacle.instance_id, obstacle.profile_id = "tag:1", "small_carton"
            obstacle.pose.pose.position.x = obstacle_x[0]
            obstacle.pose.pose.position.y, obstacle.pose.pose.position.z = 1.0, 0.14
            obstacle.pose.pose.orientation.w = 1.0
            publisher.publish(BoxStateArray(boxes=[box, obstacle]))

        def assert_obstacle(x):
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = self.read_scene.call_async(request)
            self.wait(future.done, 10.0)
            objects = {obj.id: obj for obj in future.result().scene.world.collision_objects}
            self.assertAlmostEqual(objects["grasp_box_tag_1"].pose.position.x, x, places=6)

        def status(message):
            self.statuses.append(message)
            if (message.action == injection["action"] and message.phase == injection["phase"]
                    and message.status == "running" and injection["future"] is None):
                injection["future"] = self.block(True)

        def joint_state(message):
            joints.update(zip(message.name, message.position))

        def physical_operation(request, response):
            physical_calls.append("operation")
            response.success = True
            return response

        physical_calls, services = [], []

        timer = self.node.create_timer(0.03, publish)
        subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        joint_subscription = self.node.create_subscription(JointState, "/joint_states", joint_state, 10)
        try:
            # Pick: move the selected box during a preflight planning pause.
            self.set_block(True)
            _, result, task_id = self.start(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            self.assertEqual(paused.phase, "planning_prepare")
            self.assertEqual(paused.object_disposition, "not_attached")
            position[0] = 0.45
            obstacle_x[0] = 2.2
            publish()
            self.set_block(False)
            # Replay a pre-Continue timestamp: receipt after Continue is insufficient.
            frozen_stamp[0] = self.node.get_clock().now().to_msg()
            publish()
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertIn("after the action request", paused.failure)
            self.assertEqual(physical_calls, [])
            frozen_stamp[0] = None
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertEqual(paused.phase, "attach")
            self.assertEqual(physical_calls, [])
            services.append(self.node.create_service(
                Trigger, "/mujoco_grasp/attach", physical_operation))
            # A refreshed scene must be checked at the physical checkpoint,
            # even if the target and hand contacts have not moved.
            self.set_block(True)
            self.resume(task_id, paused.pause_id)
            self.wait(lambda: self.paused(task_id, paused.pause_id) is not None)
            paused = self.paused(task_id, paused.pause_id)
            self.assertEqual(paused.phase, "attach")
            self.assertEqual(paused.object_disposition, "not_attached")
            self.assertIn("checkpoint collision", paused.failure)
            self.assertEqual(physical_calls, [])
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            self.assertTrue(self.succeed(result).object_held)
            self.assertEqual(len(physical_calls), 1)
            assert_obstacle(2.2)
            self.assertFalse(any(s.task_id == task_id and "refreshed" in s.failure
                                 for s in self.statuses))

            # Carry: retain attachment and resume the same destination.
            self.set_block(True)
            _, result, task_id = self.start(
                MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(target_pose=1))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            self.assertEqual(paused.object_disposition, "attached")
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            self.assertTrue(self.succeed(result).object_held)
            _, result, _ = self.start(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(target_pose=0))
            self.succeed(result)

            # Place: pause before release; Continue must preserve placement.
            _, result, task_id = self.start(Place, "/place_box", self.place_goal(Place))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            self.assertEqual(paused.phase, "release")
            self.assertEqual(paused.object_disposition, "attached")
            self.assertEqual(len(physical_calls), 1)
            services.append(self.node.create_service(
                Trigger, "/mujoco_grasp/detach", physical_operation))
            self.resume(task_id, paused.pause_id)
            self.assertFalse(self.succeed(result).object_held)
            self.assertEqual(len(physical_calls), 2)

            # PickPlace: Continue twice on one action, before Pick and after attach.
            # Scene updates may arrive during carry; either carry or the next
            # Place scene check must pause without repeating attachment.
            self.set_block(True)
            goal = self.place_goal(PickPlace)
            goal.place_pose.header.frame_id = "continue_place_frame"
            goal.instance_id = "tag:0"
            _, result, task_id = self.start(PickPlace, "/pick_place", goal)
            self.wait(lambda: self.paused(task_id) is not None)
            first_pause = self.paused(task_id)
            place_frame_x[0] = 0.5
            obstacle_x[0] = 2.4
            publish()
            injection.update(action="pick_place", phase="attach_scene", future=None)
            self.set_block(False)
            self.resume(task_id, first_pause.pause_id)
            self.wait(lambda: self.paused(task_id, first_pause.pause_id) is not None)
            second_pause = self.paused(task_id, first_pause.pause_id)
            self.assertEqual(second_pause.object_disposition, "attached")
            self.assertIn(second_pause.last_completed_phase, ("attach_scene", "carry"))
            self.assertIn(second_pause.phase, ("carry", "scene"))
            injection.update(action="", phase="")
            obstacle_x[0] = 2.6
            publish()
            self.set_block(False)
            self.resume(task_id, second_pause.pause_id)
            completed = self.succeed(result)
            self.assertAlmostEqual(completed.achieved_pose.pose.position.x, 0.35, delta=0.05)
            self.assertEqual(len(physical_calls), 4)
            assert_obstacle(2.6)
            checkpoints = [s.last_completed_phase for s in self.statuses if s.task_id == task_id]
            self.assertIn("release", checkpoints)
            self.assertIn("to_prepare", checkpoints)
            self.assertIn("from_prepare_to_ready", checkpoints)

            # Reset: refresh optional geometry without repeating cleanup.
            self.set_block(True)
            _, result, task_id = self.start(
                ResetManipulation, "/reset_manipulation", ResetManipulation.Goal(confirm_empty=True))
            self.wait(lambda: self.paused(task_id) is not None)
            paused = self.paused(task_id)
            self.assertEqual(paused.object_disposition, "released")
            obstacle_x[0] = 3.0
            publish()
            self.set_block(False)
            self.resume(task_id, paused.pause_id)
            self.succeed(result)
            assert_obstacle(3.0)
            srdf = Path(get_package_share_directory("agibot_x2_moveit_config")) / "config/x2_ultra.srdf"
            ready = ET.parse(srdf).find(".//group_state[@name='ready'][@group='dual_arm']")
            expected = {joint.attrib["name"]: float(joint.attrib["value"]) for joint in ready}
            self.wait(lambda: all(name in joints and abs(joints[name] - value) <= 0.1
                                  for name, value in expected.items()), 10.0)
        finally:
            for service in services:
                self.node.destroy_service(service)
            removed = self.block(False)
            self.wait(removed.done, 10.0)
            for client in self.clients:
                client.destroy()
            self.node.destroy_timer(timer)
            self.node.destroy_subscription(subscription)
            self.node.destroy_subscription(joint_subscription)
            self.node.destroy_client(self.scene)
            self.node.destroy_client(self.read_scene)
            self.node.destroy_client(self.resume_client)
            self.node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_manipulation_server_exits_cleanly(self, proc_info):
        servers = [process for process in proc_info.processes()
                   if process.process_details["name"].startswith("pick_place_server")]
        self.assertEqual(len(servers), 1)
        import launch_testing.asserts
        launch_testing.asserts.assertExitCodes(proc_info, process=servers[0])
