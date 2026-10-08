"""Exercise torso-relative carry with real X2 IK and tilted fake feedback."""

import importlib.util
import math
import os
from pathlib import Path
import time
from threading import Thread
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from aimdk_msgs.msg import JointState, JointStateArray
from agibot_x2_manipulation_msgs.action import MoveCarryPose, Pick, PickPlace
import launch_testing
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
import yaml


_spec = importlib.util.spec_from_file_location(
    "torso_carry_fixture", Path(__file__).with_name("test_dummy_workflow.launch.py"))
fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fixture)


def carry_profiles():
    profiles = yaml.safe_load((Path(__file__).parent / "config" /
                               "box_profiles_simulation.yaml").read_text())
    # Nearby test-only targets isolate frame behavior from long route searches.
    profiles["/**"]["ros__parameters"]["box_profiles"]["small_carton"]["carry_pose_b"] = [
        0.40, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
    # Require distinct nominal endpoints in this fixture rather than allowing
    # adaptive fallback to turn the A/B round trip into an empty trajectory.
    profiles["pick_place_server"] = {"ros__parameters": {
        "carry_search_x_range": 0.0,
        "carry_search_y_range": 0.0,
        "carry_search_z_lower": 0.0,
        "carry_search_z_upper": 0.0,
        "carry_search_orientation_tolerance": 0.0,
        "execution_joint_tolerance": 0.01,
    }}
    return profiles


@pytest.mark.launch_test
def generate_test_description():
    description = Path(get_package_share_directory("x2_description")) / "urdf/x2_ultra.urdf"
    joints = {joint.attrib["name"]: 0.0 for joint in ET.parse(description).getroot().findall(
        "joint") if joint.attrib["type"] != "fixed"}
    config = Path(get_package_share_directory("agibot_x2_moveit_config")) / "config"
    srdf = next(config.glob("*.srdf"))
    for joint in ET.parse(srdf).getroot().find("group_state[@name='ready']").findall("joint"):
        joints[joint.attrib["name"]] = float(joint.attrib["value"])
    for side in ("left", "right"):
        joints[f"{side}_hip_pitch_joint"] = -0.1
        joints[f"{side}_knee_joint"] = 0.3
        joints[f"{side}_ankle_pitch_joint"] = -0.2
    joints["waist_pitch_joint"] = -0.15
    state_file = Path(f"/tmp/x2_torso_carry_{os.getpid()}.yaml")
    state_file.write_text(yaml.safe_dump({"joint_positions": joints}))
    profiles_file = Path(f"/tmp/x2_torso_profiles_{os.getpid()}.yaml")
    profiles_file.write_text(yaml.safe_dump(carry_profiles()))
    return fixture.make_test_description(
        extra_arguments={"box_profiles_file": str(profiles_file)},
        extra_feedback_arguments=["--initial-state-file", str(state_file)],
        extra_feedback_remappings=[("/aima/hal/joint/waist/state", "/torso_test_fake_waist")])


class TestTorsoCarry(fixture.TestDummyWorkflow):
    def test_dummy_pick_place_and_pick_place(self):
        waist_pitch = [-0.15]
        feedback_node = rclpy.create_node("torso_carry_waist_feedback")
        waist = feedback_node.create_publisher(
            JointStateArray, "/aima/hal/joint/waist/state", qos_profile_sensor_data)

        def publish_waist():
            message = JointStateArray()
            message.header.stamp = feedback_node.get_clock().now().to_msg()
            message.header.meas_stamp = message.header.stamp
            message.joints = [JointState(name=f"waist_{axis}_joint", position=value)
                              for axis, value in (("yaw", 0.0), ("pitch", waist_pitch[0]),
                                                  ("roll", 0.0))]
            waist.publish(message)

        feedback_node.create_timer(0.02, publish_waist)
        executor = SingleThreadedExecutor()
        executor.add_node(feedback_node)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()

        def stop_feedback():
            executor.shutdown(timeout_sec=5.0)
            thread.join(timeout=5.0)
            feedback_node.destroy_node()

        self.addCleanup(stop_feedback)
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        deadline = time.monotonic() + 40.0
        while not buffer.can_transform("torso_link", "base_link", Time()):
            self.assertLess(time.monotonic(), deadline, "torso TF unavailable")
            rclpy.spin_once(self.node, timeout_sec=0.05)

        profiles = carry_profiles()
        profile = profiles["/**"]["ros__parameters"]["box_profiles"]["small_carton"]
        parameters = yaml.safe_load((Path(get_package_share_directory(
            "agibot_x2_manipulation")) / "config/box_manipulation.yaml").read_text())
        limits = parameters["pick_place_server"]["ros__parameters"]

        def check_target(result, target):
            self.assertEqual(result.achieved_pose.header.frame_id, "base_link")
            actual = do_transform_pose(result.achieved_pose.pose, buffer.lookup_transform(
                "torso_link", "base_link", Time()))
            self.assertLessEqual(abs(actual.position.x - target[0]),
                                 limits["carry_search_x_range"] + 1e-4)
            self.assertLessEqual(abs(actual.position.y - target[1]),
                                 limits["carry_search_y_range"] + 1e-4)
            self.assertGreaterEqual(actual.position.z - target[2],
                                    -limits["carry_search_z_lower"] - 1e-4)
            self.assertLessEqual(actual.position.z - target[2],
                                 limits["carry_search_z_upper"] + 1e-4)
            angle = 2 * math.acos(min(1.0, abs(actual.orientation.w)))
            self.assertLessEqual(angle, limits["carry_search_orientation_tolerance"] + 1e-4)

        pick = self.send_goal(Pick, "/pick_box", Pick.Goal(
            instance_id="tag:0", plan_only=True), 100.0)
        check_target(pick, profile["carry_pose_a"])
        combined = PickPlace.Goal(instance_id="tag:0", plan_only=True)
        self.place_pose(combined)
        self.send_goal(PickPlace, "/pick_place", combined, 150.0)
        pick = self.send_goal(Pick, "/pick_box", Pick.Goal(
            instance_id="tag:0", plan_only=True), 100.0)
        executed = self.send_goal(Pick, "/pick_box", Pick.Goal(
            instance_id="tag:0", plan_id=pick.plan_id), 120.0)
        check_target(executed, profile["carry_pose_a"])
        for index, target in enumerate((MoveCarryPose.Goal.CARRY_A, MoveCarryPose.Goal.CARRY_B,
                                        MoveCarryPose.Goal.CARRY_A)):
            preview = self.send_goal(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(
                target_pose=target, plan_only=True), 100.0)
            if index == 0:
                self.assertIn("already at carry pose", preview.message)
            result = self.send_goal(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(
                target_pose=target, plan_id=preview.plan_id), 100.0)
            nominal = profile["carry_pose_a" if target == MoveCarryPose.Goal.CARRY_A
                              else "carry_pose_b"]
            check_target(preview, nominal)
            check_target(result, nominal)
        stale = self.send_goal(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(
            target_pose=MoveCarryPose.Goal.CARRY_B, plan_only=True), 100.0)
        waist_pitch[0] = 0.1
        deadline = time.monotonic() + 10.0
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            rotation = buffer.lookup_transform("base_link", "torso_link", Time()).transform.rotation
            if abs(rotation.y - math.sin(0.1 / 2)) < 1e-4:
                break
            self.assertLess(time.monotonic(), deadline, "waist feedback did not update torso TF")
        # Give the independent MoveIt state monitor time to consume the same feedback.
        settle = time.monotonic() + 0.5
        while time.monotonic() < settle:
            rclpy.spin_once(self.node, timeout_sec=0.03)
        client = ActionClient(self.node, MoveCarryPose, "/move_carry_pose")
        sent = client.send_goal_async(MoveCarryPose.Goal(
            target_pose=MoveCarryPose.Goal.CARRY_B, plan_id=stale.plan_id))
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        self.assertTrue(sent.done())
        self.assertTrue(sent.result().accepted)
        finished = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, finished, timeout_sec=10.0)
        self.assertTrue(finished.done())
        self.assertFalse(finished.result().result.success)
        self.assertIn("torso posture changed", finished.result().result.message)
        client.destroy()
        # A posture-only change moves both hands with the torso. Returning to
        # remembered A must need no arm motion, and no recovery confirmation.
        fresh = self.send_goal(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(
            target_pose=MoveCarryPose.Goal.CARRY_A, plan_only=True), 100.0)
        self.assertIn("already at carry pose", fresh.message)
        result = self.send_goal(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(
            target_pose=MoveCarryPose.Goal.CARRY_A, plan_id=fresh.plan_id), 100.0)
        check_target(fresh, profile["carry_pose_a"])
        check_target(result, profile["carry_pose_a"])
        listener.unregister()
