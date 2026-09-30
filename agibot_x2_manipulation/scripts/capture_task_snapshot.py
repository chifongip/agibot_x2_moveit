#!/usr/bin/env python3
"""Capture a complete stationary pre-task state, or replay its tag observations."""

import argparse
from collections import deque
import math
from pathlib import Path
import sys
import time

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import TransformBroadcaster, TransformException
import yaml

from capture_failure_snapshot import FailureSnapshotRecorder, transform_to_dict


X2_JOINT_NAMES = {
    f"{side}_{part}_joint"
    for side in ("left", "right")
    for part in (
        "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll",
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
        "wrist_yaw", "wrist_pitch", "wrist_roll",
    )
} | {f"waist_{axis}_joint" for axis in ("yaw", "pitch", "roll")} | {
    "head_yaw_joint", "head_pitch_joint",
}


def parse_arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--output", help="New snapshot YAML path; never overwritten.")
    mode.add_argument("--replay", help="Publish saved tag poses for offline simulation.")
    parser.add_argument("--tag-id", action="append", type=int)
    parser.add_argument("--object-id", default="tag:0")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--box-states-topic", default="/box_states")
    parser.add_argument("--detections-topic", action="append")
    parser.add_argument("--tag-frame-prefix", default="tag")
    parser.add_argument("--planning-frame", default="base_link")
    parser.add_argument("--robot-pose-parent-frame", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-age", type=float, default=1.0)
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    args.tag_id = args.tag_id if args.tag_id is not None else [0, 9]
    if any(tag < 0 for tag in args.tag_id):
        parser.error("tag IDs must be non-negative")
    if not args.tag_frame_prefix or not args.planning_frame or not args.object_id:
        parser.error("frame names and object ID must not be empty")
    for name in ("timeout", "max_age"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    args.capture_once = False
    args.action_status_topic = []
    args.waiting_message = "Waiting for complete pre-task robot/object state"
    return args


def capture_readiness(capture, required_tags, object_id, now_ros, max_age):
    """Reject incomplete/stale data rather than produce a misleading replay."""
    robot = capture["robot"]
    joint = robot.get("joint_state", {})
    positions = joint.get("joint_positions", {})
    if set(positions) != X2_JOINT_NAMES:
        return "waiting for all 31 finite X2 joint positions"
    stamp = joint["stamp"]
    age = now_ros - stamp["sec"] - stamp["nanosec"] * 1e-9
    if age < -0.1 or age > max_age:
        return "waiting for a fresh joint-state timestamp"
    boxes = {box["instance_id"]: box for box in capture["visible_box_states"]}
    if object_id not in boxes:
        return f"waiting for object {object_id}"
    box = boxes[object_id]
    stamp = box["stamp"]
    age = now_ros - stamp["sec"] - stamp["nanosec"] * 1e-9
    if age < -0.1 or age > max_age:
        return f"waiting for a fresh pose of {object_id}"
    if not valid_pose(box["pose"]["position"], box["pose"]["orientation"]):
        return f"waiting for a finite pose of {object_id}"
    tags = {tag["tag_id"]: tag for tag in capture["tags"]}
    for tag_id in required_tags:
        transform = tags.get(tag_id, {}).get("pose", {}).get("transform")
        if transform is None:
            error = tags.get(tag_id, {}).get("pose", {}).get("error", "no transform received")
            return f"waiting for TF of tag {tag_id}: {error}"
        stamp = transform["stamp"]
        transform_time = stamp["sec"] + stamp["nanosec"] * 1e-9
        # Zero is the conventional stamp for a fully static TF chain.
        if transform_time != 0.0:
            age = now_ros - transform_time
            joint_stamp = joint["stamp"]
            joint_time = joint_stamp["sec"] + joint_stamp["nanosec"] * 1e-9
            if age < -0.1 or age > max_age or abs(transform_time - joint_time) > max_age:
                return f"waiting for a fresh TF of tag {tag_id}"
        if not valid_pose(transform["translation"], transform["rotation"]):
            return f"waiting for a finite TF of tag {tag_id}"
    if "base_pose" in robot and "transform" not in robot["base_pose"]:
        return "waiting for the requested robot base transform"
    return None


def valid_pose(position, rotation):
    values = list(position.values()) + list(rotation.values())
    return all(math.isfinite(v) for v in values) and sum(
        v * v for v in rotation.values()
    ) > 1e-12


class TaskSnapshotRecorder(FailureSnapshotRecorder):
    def __init__(self, args):
        super().__init__(args)
        self.joint_samples = deque(maxlen=200)
        self.required_tags = set(args.tag_id)
        self.object_id = args.object_id
        self.max_age = args.max_age
        self.timeout = args.timeout
        self.started_monotonic = time.monotonic()
        self.exit_code = 0
        self.waiting_for = "waiting for state"
        self.timer = self.create_timer(0.1, self.try_capture)
        self.get_logger().info(
            "Keep the robot and objects stationary. Start Pick/Place only after "
            "this command exits successfully."
        )

    def on_joint_state(self, message):
        self.joint_samples.append(message)

    def on_action_status(self, message, source_topic):
        # Pre-task recording is independent of old action status messages.
        pass

    def lookup_transform(self, parent_frame, child_frame):
        # Prefer interpolation, but stationary captures also support delayed or
        # sparse camera TFs. Readiness checks still enforce the freshness limit.
        query_time = Time.from_msg(self.latest_joint_state.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, query_time)
            return {"transform": transform_to_dict(transform), "lookup_mode": "joint_state_time"}
        except TransformException as exact_error:
            try:
                transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, Time())
                return {
                    "transform": transform_to_dict(transform),
                    "lookup_mode": "latest_available",
                    "joint_time_lookup_error": str(exact_error),
                }
            except TransformException as latest_error:
                return {"error": str(latest_error), "joint_time_lookup_error": str(exact_error)}

    def build_capture(self, reason, action_status=None):
        # Only required tags are replayed; cached transient detections are diagnostic.
        capture = super().build_capture(reason, action_status)
        capture["tags"] = [
            tag for tag in capture["tags"] if tag["tag_id"] in self.required_tags
        ]
        capture["capture"]["object_id"] = self.object_id
        return capture

    def try_capture(self):
        if self.capture_complete:
            return
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        # Give asynchronous TF publishers time to bracket the selected sample.
        eligible = [
            sample for sample in self.joint_samples
            if Time.from_msg(sample.header.stamp).nanoseconds * 1e-9 <= now_ros - 0.15
        ]
        if eligible:
            self.latest_joint_state = eligible[-1]
        if self.latest_joint_state is not None:
            capture = self.build_capture("before_task")
            self.waiting_for = capture_readiness(
                capture, self.required_tags, self.object_id,
                now_ros, self.max_age,
            )
            if self.waiting_for is None:
                self.output_path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with self.output_path.open("x", encoding="utf-8") as stream:
                        yaml.safe_dump(capture, stream, sort_keys=False)
                except OSError as error:
                    self.get_logger().error(str(error))
                    self.exit_code = 1
                else:
                    self.get_logger().info(f"Wrote pre-task snapshot: {self.output_path}")
                self.capture_complete = True
                return
        if time.monotonic() - self.started_monotonic >= self.timeout:
            self.get_logger().error(f"Capture timed out: {self.waiting_for}")
            self.exit_code = 1
            self.capture_complete = True


class SnapshotTagReplay(Node):
    """Republish frozen transforms/detections using current simulation time."""

    def __init__(self, path):
        super().__init__("snapshot_tag_replay")
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            snapshot = yaml.safe_load(stream)
        self.tags = snapshot["tags"]
        if not self.tags or any("transform" not in tag["pose"] for tag in self.tags):
            raise ValueError("snapshot must contain valid tag transforms")
        self.planning_frame = snapshot["robot"]["planning_frame"]
        self.publisher = self.create_publisher(
            AprilTagDetectionArray, "/detections", qos_profile_sensor_data
        )
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30.0, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        detections = AprilTagDetectionArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.planning_frame
        transforms = []
        for tag in self.tags:
            saved = tag["pose"]["transform"]
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = saved["parent_frame"]
            transform.child_frame_id = saved["child_frame"]
            for axis in ("x", "y", "z"):
                setattr(transform.transform.translation, axis, saved["translation"][axis])
            for axis in ("x", "y", "z", "w"):
                setattr(transform.transform.rotation, axis, saved["rotation"][axis])
            transforms.append(transform)
            detection = AprilTagDetection()
            detection.id = tag["tag_id"]
            detection.family = tag.get("latest_detection", {}).get("family", "tag36h11")
            detection.hamming = 0
            detection.goodness = 1.0
            detection.decision_margin = 100.0
            detections.detections.append(detection)
        self.broadcaster.sendTransform(transforms)
        self.publisher.publish(detections)


def main(argv=None):
    argv = sys.argv if argv is None else argv
    args = parse_arguments(argv)
    rclpy.init(args=argv)
    node = None
    result = 0
    try:
        if args.replay:
            node = SnapshotTagReplay(args.replay)
            rclpy.spin(node)
        else:
            node = TaskSnapshotRecorder(args)
            while rclpy.ok() and not node.capture_complete:
                rclpy.spin_once(node, timeout_sec=0.1)
            result = node.exit_code
    except KeyboardInterrupt:
        # An interrupted capture must not silently save an incomplete state.
        result = 130
    except (OSError, ValueError, KeyError) as error:
        print(f"Snapshot error: {error}", file=sys.stderr)
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
