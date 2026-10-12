"""Resume post-release retreat without replaying release, including saved plans."""

import importlib.util
from pathlib import Path
import tempfile

from agibot_x2_manipulation_msgs.action import Pick, Place
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray, ManipulationTaskStatus
from agibot_x2_manipulation_msgs.srv import ContinueManipulation
import launch_testing
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import yaml


spec = importlib.util.spec_from_file_location(
    "retreat_continue_fixture", Path(__file__).with_name("test_continue_actions.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    profiles["pick_place_server"] = {"ros__parameters": {"simulate_ideal_attachment": True}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as params:
        yaml.safe_dump(profiles, params)
    return fixture.fixture.make_test_description(params.name, extra_arguments={
        "phase_retry_attempts": "1", "phase_retry_timeout": "20.0"})


class TestRetreatContinue(fixture.TestContinueActions):
    def test_continue_all_actions(self):
        rclpy.init()
        self.node = rclpy.create_node("retreat_continue_test")
        self.statuses, self.clients = [], []
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.resume_client = self.node.create_client(ContinueManipulation, "/continue_manipulation")
        read_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.assertTrue(self.scene.wait_for_service(timeout_sec=40.0))
        self.assertTrue(self.resume_client.wait_for_service(timeout_sec=40.0))
        self.assertTrue(read_scene.wait_for_service(timeout_sec=40.0))
        publisher = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        physical = []
        obstacle_x = [2.0]

        def publish():
            boxes = []
            for tag, x, y in ((0, 0.33, 0.0), (1, obstacle_x[0], 1.0)):
                box = BoxState(instance_id=f"tag:{tag}", profile_id="small_carton")
                box.header.frame_id = "base_link"
                box.header.stamp = self.node.get_clock().now().to_msg()
                box.pose.pose.position.x = x
                box.pose.pose.position.y = y
                box.pose.pose.position.z = 0.14
                box.pose.pose.orientation.w = 1.0
                boxes.append(box)
            publisher.publish(BoxStateArray(boxes=boxes))

        def attach(request, response):
            physical.append("attach")
            response.success = True
            return response

        async def release(request, response):
            physical.append("release")
            # Install the obstacle before the release response: the next
            # unfinished motion must pause even if a return was previewed.
            blocked = await self.block(True)
            self.assertTrue(blocked.success)
            response.success = True
            return response

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = self.node.create_subscription(
            ManipulationTaskStatus, "/manipulation_task_status", self.statuses.append, qos)
        services = [self.node.create_service(Trigger, "/mujoco_grasp/attach", attach),
                    self.node.create_service(Trigger, "/mujoco_grasp/detach", release,
                                             callback_group=ReentrantCallbackGroup())]
        timer = self.node.create_timer(0.03, publish)
        try:
            for saved in (False, True):
                _, result, _ = self.start(Pick, "/pick_box", Pick.Goal(instance_id="tag:0"))
                self.succeed(result)
                goal = self.place_goal(Place)
                if saved:
                    goal.plan_only = True
                    _, result, _ = self.start(Place, "/place_box", goal)
                    preview = self.succeed(result)
                    goal = Place.Goal(plan_id=preview.plan_id)
                _, result, task = self.start(Place, "/place_box", goal)
                self.wait(lambda: self.paused(task) is not None)
                paused = self.paused(task)
                self.assertIn("retreat", paused.phase)
                self.assertEqual(paused.object_disposition, "released")
                before = list(physical)
                self.resume(task, paused.pause_id)
                self.wait(lambda: self.paused(task, paused.pause_id) is not None)
                paused = self.paused(task, paused.pause_id)
                self.assertEqual(physical, before)
                self.assertEqual(paused.object_disposition, "released")
                self.set_block(False)
                obstacle_x[0] += 0.2
                publish()
                self.resume(task, paused.pause_id)
                self.succeed(result)
                self.assertEqual(physical, before)
                checkpoints = [s.last_completed_phase for s in self.statuses if s.task_id == task]
                expected = ("saved/retreat", "saved/prepare_direct", "saved/ready_direct") if saved else (
                    "retreat", "to_prepare", "from_prepare_to_ready")
                for checkpoint in expected:
                    self.assertIn(checkpoint, checkpoints)
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                future = read_scene.call_async(request)
                self.wait(future.done, 10.0)
                objects = {o.id: o for o in future.result().scene.world.collision_objects}
                self.assertAlmostEqual(objects["grasp_box_tag_1"].pose.position.x,
                                       obstacle_x[0], places=6)
            self.assertEqual(physical, ["attach", "release"] * 2)
        finally:
            self.node.destroy_timer(timer)
            for service in services:
                self.node.destroy_service(service)
            self.node.destroy_subscription(subscription)
            for client in self.clients:
                client.destroy()
            for client in (self.scene, self.resume_client, read_scene):
                self.node.destroy_client(client)
            self.node.destroy_publisher(publisher)
            self.node.destroy_node()
            rclpy.shutdown()
