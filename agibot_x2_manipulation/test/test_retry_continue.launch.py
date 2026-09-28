"""Exercise the real server's retry/pause/Continue lifecycle with simulated arms."""

import os
from pathlib import Path
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick, ResetManipulation
from agibot_x2_manipulation_msgs.msg import ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
from geometry_msgs.msg import Pose
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    port = 40000 + os.getpid() % 10000
    state_file = f"/tmp/x2_retry_continue_{os.getpid()}"
    Path(state_file + ".task").write_text(
        '"past-task" "reset" "canceled" "reset_to_ready" "reset_scene" "released" "canceled"\n',
        encoding="utf-8",
    )
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "recorded_planning_failure.launch.py")),
        launch_arguments={
            "use_rviz": "false", "allow_execution": "true",
            "zmq_endpoint": f"tcp://*:{port}", "fake_zmq_endpoint": f"tcp://127.0.0.1:{port}",
            "motion_planning_mode": LaunchConfiguration("mode"),
            "phase_retry_attempts": "2", "phase_retry_timeout": "10.0", "phase_retry_delay": "1.0",
            "manipulation_state_file": state_file,
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="pose_to_pose"),
        stack, launch_testing.actions.ReadyToTest(),
    ])


class TestRetryContinue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.node = rclpy.create_node("retry_continue_test")
        cls.statuses = []
        cls.subscription = cls.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", cls.statuses.append,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        cls.reset_client = ActionClient(cls.node, ResetManipulation, "/reset_manipulation")
        cls.continue_client = cls.node.create_client(ContinueManipulation, "/continue_manipulation")
        cls.scene_client = cls.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    @classmethod
    def tearDownClass(cls):
        cls.reset_client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, timeout=35.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), [(s.status, s.phase, s.failure) for s in self.statuses[-5:]])

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=40.0))
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def blocker(self, present):
        obj = CollisionObject()
        obj.id = "retry_test_external_blocker"
        obj.header.frame_id = "base_link"
        obj.operation = CollisionObject.ADD if present else CollisionObject.REMOVE
        if present:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [4.0, 4.0, 4.0]
            pose = Pose()
            pose.orientation.w = 1.0
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [obj]
        self.assertTrue(self.call(self.scene_client, request).success)

    def start(self):
        self.assertTrue(self.reset_client.wait_for_server(timeout_sec=40.0))
        goal = ResetManipulation.Goal()
        goal.confirm_empty = True
        future = self.reset_client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        task_id = bytes(handle.goal_id.uuid).hex()
        return handle, handle.get_result_async(), task_id

    def status(self, task_id, name):
        return next((s for s in reversed(self.statuses) if s.task_id == task_id and s.status == name), None)

    def resume(self, task_id, pause_id):
        request = ContinueManipulation.Request()
        request.task_id = task_id
        request.pause_id = pause_id
        return self.call(self.continue_client, request)

    def test_01_single_failure_retries_without_ending_action(self):
        self.wait(lambda: self.status("past-task", "canceled") is not None)
        self.assertFalse(self.status("past-task", "canceled").can_continue)
        self.blocker(True)
        _, result, task_id = self.start()
        self.wait(lambda: any(s.task_id == task_id and s.failure for s in self.statuses))
        self.assertFalse(result.done())
        self.blocker(False)
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED, result.result().result.message)
        self.assertTrue(any(s.task_id == task_id and s.attempt == 2 for s in self.statuses))
        self.assertIsNone(self.status(task_id, "paused"))

    def test_02_continue_resumes_same_action_and_rejects_stale_requests(self):
        self.blocker(True)
        _, result, task_id = self.start()
        self.wait(lambda: self.status(task_id, "paused") is not None)
        paused = self.status(task_id, "paused")
        self.assertEqual(paused.maximum_attempts, 2)
        self.assertEqual(paused.attempt, 2)
        self.assertTrue(paused.can_continue)
        self.assertEqual(paused.object_disposition, "released")
        self.assertFalse(result.done())
        pick = ActionClient(self.node, Pick, "/pick_box")
        self.assertTrue(pick.wait_for_server(timeout_sec=5.0))
        refused = pick.send_goal_async(Pick.Goal(plan_only=True))
        self.wait(refused.done)
        self.assertFalse(refused.result().accepted)
        pick.destroy()
        self.assertFalse(self.resume("other", paused.pause_id).success)
        self.assertFalse(self.resume(task_id, paused.pause_id - 1).success)
        self.blocker(False)
        self.assertTrue(self.resume(task_id, paused.pause_id).success)
        self.assertFalse(self.resume(task_id, paused.pause_id).success)
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED, result.result().result.message)

    def test_03_cancel_paused_action(self):
        self.blocker(True)
        handle, result, task_id = self.start()
        self.wait(lambda: self.status(task_id, "paused") is not None)
        canceled = handle.cancel_goal_async()
        self.wait(canceled.done)
        self.assertTrue(canceled.result().goals_canceling)
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.wait(lambda: self.status(task_id, "canceled") is not None)
        self.assertFalse(self.status(task_id, "canceled").can_continue)
        self.blocker(False)


    def test_04_shutdown_while_paused(self):
        self.blocker(True)
        _, result, task_id = self.start()
        self.wait(lambda: self.status(task_id, "paused") is not None)
        self.assertFalse(result.done())
        # Leave this worker paused so launch teardown verifies its lifetime handling.


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_manipulation_server_exits_cleanly(self, proc_info):
        servers = [process for process in proc_info.processes()
                   if process.process_details["name"].startswith("pick_place_server")]
        self.assertEqual(len(servers), 1)
        import launch_testing.asserts
        launch_testing.asserts.assertExitCodes(proc_info, process=servers[0])
