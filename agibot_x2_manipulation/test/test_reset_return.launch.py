import os
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import ResetManipulation
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
from geometry_msgs.msg import Pose
import pytest
import rclpy
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    port = 30000 + os.getpid() % 10000
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "recorded_planning_failure.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "zmq_endpoint": f"tcp://*:{port}",
            "fake_zmq_endpoint": f"tcp://127.0.0.1:{port}",
            "allow_execution": "true",
            "motion_planning_mode": "pose_to_pose",
            "manipulation_state_file": f"/tmp/x2_reset_return_{os.getpid()}",
        }.items(),
    )
    return LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestResetReturn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("reset_return_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=40.0), name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), name)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def obstacle(self, name, position, dimensions):
        object_msg = CollisionObject()
        object_msg.id = name
        object_msg.header.frame_id = "base_link"
        object_msg.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        object_msg.primitives = [box]
        object_msg.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [object_msg]
        self.assertTrue(self.call(ApplyPlanningScene, "/apply_planning_scene", request).success)

    def reset(self, confirm=True):
        client = ActionClient(self.node, ResetManipulation, "/reset_manipulation")
        self.assertTrue(client.wait_for_server(timeout_sec=40.0))
        goal = ResetManipulation.Goal()
        goal.confirm_empty = confirm
        stages = []
        future = client.send_goal_async(
            goal, feedback_callback=lambda feedback: stages.append(feedback.feedback.stage)
        )
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        handle = future.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=75.0)
        self.assertTrue(result_future.done())
        result = result_future.result()
        client.destroy()
        return result, stages

    def test_reset_clears_stale_detections_and_rejects_external_blocker(self):
        refused, _ = self.reset(confirm=False)
        self.assertEqual(refused.result.error_code, ResetManipulation.Result.CONFIRMATION_REQUIRED)
        self.obstacle("work_table", [0.4, 0.0, -0.29], [0.5, 0.3, 0.6])
        self.obstacle("grasp_box_reset_obstacle", [0.35, 0.0, 0.17], [0.15, 0.36, 0.32])
        self.obstacle("external_obstacle", [-1.0, 0.0, 0.5], [0.1, 0.1, 0.1])
        completed, stages = self.reset()
        self.assertEqual(completed.status, GoalStatus.STATUS_SUCCEEDED, completed.result.message)
        self.assertTrue(completed.result.success)
        self.assertIn("planning_zero", stages)
        self.assertIn("executing_zero", stages)
        self.assertIn("verifying", stages)
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE
        )
        scene = self.call(GetPlanningScene, "/get_planning_scene", request).scene
        ids = {object_msg.id for object_msg in scene.world.collision_objects}
        self.assertNotIn("work_table", ids)
        self.assertNotIn("grasp_box_reset_obstacle", ids)
        self.assertIn("external_obstacle", ids)
        fk = GetPositionFK.Request()
        fk.header.frame_id = "base_link"
        fk.fk_link_names = ["left_wrist_roll_link"]
        fk.robot_state = scene.robot_state
        response = self.call(GetPositionFK, "/compute_fk", fk)
        self.assertEqual(response.error_code.val, response.error_code.SUCCESS)
        wrist = response.pose_stamped[0].pose.position
        self.obstacle("reset_blocker", [wrist.x, wrist.y, wrist.z], [0.1, 0.1, 0.1])
        blocked, stages = self.reset()
        self.assertEqual(blocked.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(blocked.result.error_code, ResetManipulation.Result.PLANNING_FAILED)
        self.assertNotIn("executing_zero", stages)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
