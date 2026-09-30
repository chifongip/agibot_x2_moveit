"""Resume Place's released-object return and verify the final Ready joint state."""

import importlib.util
from pathlib import Path
import time
import unittest
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
from geometry_msgs.msg import Pose
import launch_testing
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

_spec = importlib.util.spec_from_file_location(
    "place_continue_workflow", Path(__file__).with_name("test_dummy_workflow.launch.py"))
workflow = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(workflow)


@pytest.mark.launch_test
def generate_test_description():
    return workflow.generate_test_description()


class TestPlaceContinue(workflow.TestDummyWorkflow):
    def wait(self, predicate, timeout=90.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), [(s.phase, s.status, s.failure) for s in self.statuses[-8:]])

    @staticmethod
    def blocker_request(present):
        obj = CollisionObject()
        obj.id = "place_continue_blocker"
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
        return request

    def test_dummy_pick_place_and_pick_place(self):
        self.statuses = []
        joints = {}
        scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        resume = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        self.assertTrue(scene.wait_for_service(timeout_sec=40.0))
        self.assertTrue(resume.wait_for_service(timeout_sec=40.0))
        blocking = None

        def status(message):
            nonlocal blocking
            self.statuses.append(message)
            # Inject a return failure only after the physical release checkpoint.
            if (message.action == "place" and message.object_disposition == "released"
                    and message.last_completed_phase == "release" and blocking is None):
                blocking = scene.call_async(self.blocker_request(True))

        def joint_state(message):
            joints.update(zip(message.name, message.position))

        status_subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", status,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        joint_subscription = self.node.create_subscription(JointState, "/joint_states", joint_state, 10)
        try:
            picked = self.send_goal(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"), 90.0)
            self.assertTrue(picked.object_held)
            client = ActionClient(self.node, Place, "/place_box")
            self.assertTrue(client.wait_for_server(timeout_sec=10.0))
            goal = Place.Goal()
            self.place_pose(goal)
            sent = client.send_goal_async(goal)
            self.wait(sent.done)
            handle = sent.result()
            self.assertTrue(handle.accepted)
            task_id = bytes(handle.goal_id.uuid).hex()
            result = handle.get_result_async()
            self.wait(lambda: blocking is not None and blocking.done())
            self.assertTrue(blocking.result().success)
            self.wait(lambda: any(s.task_id == task_id and s.status == "paused" for s in self.statuses))
            paused = next(s for s in reversed(self.statuses)
                          if s.task_id == task_id and s.status == "paused")
            self.assertEqual(paused.object_disposition, "released")
            self.assertFalse(result.done())
            removed = scene.call_async(self.blocker_request(False))
            self.wait(removed.done)
            self.assertTrue(removed.result().success)
            continued = resume.call_async(ContinueManipulation.Request(
                task_id=task_id, pause_id=paused.pause_id))
            self.wait(continued.done)
            self.assertTrue(continued.result().success)
            self.wait(result.done)
            wrapped = result.result()
            self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED, wrapped.result.message)
            self.assertTrue(wrapped.result.success, wrapped.result.message)
            self.assertFalse(wrapped.result.object_held)
            checkpoints = [s.last_completed_phase for s in self.statuses if s.task_id == task_id]
            self.assertIn("to_prepare", checkpoints)
            self.assertIn("from_prepare_to_ready", checkpoints)
            # Action success alone would miss the original early-completion bug.
            srdf = Path(get_package_share_directory("agibot_x2_moveit_config")) / "config/x2_ultra.srdf"
            ready = ET.parse(srdf).find(".//group_state[@name='ready'][@group='dual_arm']")
            expected = {joint.attrib["name"]: float(joint.attrib["value"]) for joint in ready}
            self.wait(lambda: all(name in joints and abs(joints[name] - value) <= 0.1
                                  for name, value in expected.items()), timeout=10.0)
            client.destroy()
        finally:
            removed = scene.call_async(self.blocker_request(False))
            rclpy.spin_until_future_complete(self.node, removed, timeout_sec=5.0)
            self.node.destroy_subscription(status_subscription)
            self.node.destroy_subscription(joint_subscription)
            self.node.destroy_client(scene)
            self.node.destroy_client(resume)


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_manipulation_server_exits_cleanly(self, proc_info):
        servers = [process for process in proc_info.processes()
                   if process.process_details["name"].startswith("pick_place_server")]
        self.assertEqual(len(servers), 1)
        import launch_testing.asserts
        launch_testing.asserts.assertExitCodes(proc_info, process=servers[0])
