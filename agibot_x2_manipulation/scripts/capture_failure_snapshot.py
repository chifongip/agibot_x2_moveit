#!/usr/bin/env python3
"""Capture the robot configuration and tag poses when manipulation aborts.

The recorder is intentionally passive: it never commands the robot or changes
the planning scene.  Start it before a Pick, Place, PickPlace, or carry action.
On an aborted action it writes one YAML file containing the latest measured
joint state, visible box states, AprilTag observations, and base-to-tag TFs.
Pressing Ctrl-C writes the same snapshot manually when an action is not used.
"""

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import BoxStateArray
from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


DEFAULT_ACTION_STATUS_TOPICS = (
    "/pick_box/_action/status",
    "/place_box/_action/status",
    "/pick_place/_action/status",
    "/move_carry_pose/_action/status",
)


def stamp_to_dict(stamp):
    """Return a ROS time message as YAML-safe integer fields."""
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def pose_to_dict(pose):
    """Return a geometry pose as a YAML-safe mapping."""
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def transform_to_dict(transform):
    """Return a TransformStamped as a YAML-safe mapping."""
    return {
        "parent_frame": transform.header.frame_id,
        "child_frame": transform.child_frame_id,
        "stamp": stamp_to_dict(transform.header.stamp),
        "translation": {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "z": float(transform.transform.translation.z),
        },
        "rotation": {
            "x": float(transform.transform.rotation.x),
            "y": float(transform.transform.rotation.y),
            "z": float(transform.transform.rotation.z),
            "w": float(transform.transform.rotation.w),
        },
    }


def joint_state_to_dict(message):
    """Serialize a joint-state message and identify unusable position values."""
    positions = {}
    invalid_positions = []
    for name, value in zip(message.name, message.position):
        value = float(value)
        if math.isfinite(value):
            positions[name] = value
        else:
            invalid_positions.append(name)

    result = {
        "topic": None,
        "frame_id": message.header.frame_id,
        "stamp": stamp_to_dict(message.header.stamp),
        "joint_positions": dict(sorted(positions.items())),
    }
    if invalid_positions:
        result["invalid_position_joints"] = sorted(invalid_positions)
    return result


def box_state_to_dict(state, received_at):
    """Serialize one localized box state without relying on ROS YAML support."""
    return {
        "instance_id": state.instance_id,
        "profile_id": state.profile_id,
        "frame_id": state.header.frame_id,
        "stamp": stamp_to_dict(state.header.stamp),
        "received_at_unix": received_at,
        "pose": pose_to_dict(state.pose.pose),
        "covariance": [float(value) for value in state.pose.covariance],
    }


def tag_id_from_instance(instance_id):
    """Extract a numeric tag ID from the stable ``tag:<id>`` instance format."""
    prefix = "tag:"
    if not instance_id.startswith(prefix):
        return None
    suffix = instance_id[len(prefix):]
    if not suffix.isdigit():
        return None
    return int(suffix)


def goal_id_to_hex(goal_id):
    """Return the ROS action UUID in a portable, log-friendly form."""
    return "".join(f"{value:02x}" for value in goal_id.uuid)


def parse_arguments(argv):
    """Parse recorder options while allowing ordinary ROS arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Record the measured robot joint state and visible AprilTag poses "
            "when a manipulation action aborts."
        )
    )
    parser.add_argument(
        "--output",
        required=True,
        help="New YAML file to create. The recorder refuses to overwrite an existing file.",
    )
    parser.add_argument(
        "--joint-state-topic",
        default="/joint_states",
        help="Measured robot joint-state topic (default: %(default)s).",
    )
    parser.add_argument(
        "--box-states-topic",
        default="/box_states",
        help="Localized box-state topic (default: %(default)s).",
    )
    parser.add_argument(
        "--detections-topic",
        action="append",
        default=None,
        help=(
            "AprilTag detection topic to observe. Repeat for additional cameras. "
            "Defaults to /detections."
        ),
    )
    parser.add_argument(
        "--action-status-topic",
        action="append",
        default=None,
        help=(
            "Action status topic that triggers a capture on STATUS_ABORTED. "
            "Repeat to override the default manipulation action topics."
        ),
    )
    parser.add_argument(
        "--tag-id",
        action="append",
        type=int,
        default=[],
        help=(
            "Tag ID to include even when it is not in /box_states or a detection. "
            "Repeat for each configured pickup or table tag."
        ),
    )
    parser.add_argument(
        "--tag-frame-prefix",
        default="tag",
        help="Prefix used to form tag TF frame names (default: %(default)s).",
    )
    parser.add_argument(
        "--planning-frame",
        default="base_link",
        help="Frame in which tag poses are recorded (default: %(default)s).",
    )
    parser.add_argument(
        "--robot-pose-parent-frame",
        default="",
        help=(
            "Optional parent frame for a world-to-base transform, for example odom. "
            "Leave empty when joint positions and planning-frame tag poses are sufficient."
        ),
    )
    parser.add_argument(
        "--capture-once",
        action="store_true",
        help=(
            "Capture after the first measured joint state instead of waiting for "
            "an action abort."
        ),
    )
    arguments = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not arguments.tag_frame_prefix:
        parser.error("--tag-frame-prefix must not be empty")
    if any(tag_id < 0 for tag_id in arguments.tag_id):
        parser.error("--tag-id values must be non-negative")
    return arguments


class FailureSnapshotRecorder(Node):
    """Passively collect the latest state and write it once on failure."""

    def __init__(self, arguments):
        super().__init__("failure_snapshot_recorder")
        self.output_path = Path(arguments.output).expanduser()
        if self.output_path.exists():
            raise ValueError(f"refusing to overwrite existing capture: {self.output_path}")
        self.joint_state_topic = arguments.joint_state_topic
        self.box_states_topic = arguments.box_states_topic
        self.planning_frame = arguments.planning_frame
        self.robot_pose_parent_frame = arguments.robot_pose_parent_frame
        self.tag_frame_prefix = arguments.tag_frame_prefix
        self.capture_once = arguments.capture_once
        self.capture_complete = False
        self.latest_joint_state = None
        self.latest_box_states = {}
        self.latest_detections = {}
        self.tag_ids = set(arguments.tag_id)
        self.capture_started_unix = time.time()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_state_topic, self.on_joint_state, qos_profile_sensor_data
        )
        self.box_states_sub = self.create_subscription(
            BoxStateArray, self.box_states_topic, self.on_box_states, qos_profile_sensor_data
        )
        detection_topics = arguments.detections_topic or ["/detections"]
        self.detection_subscriptions = [
            self.create_subscription(
                AprilTagDetectionArray,
                topic,
                lambda message, source=topic: self.on_detections(message, source),
                qos_profile_sensor_data,
            )
            for topic in detection_topics
        ]
        status_topics = arguments.action_status_topic or DEFAULT_ACTION_STATUS_TOPICS
        self.status_subscriptions = [
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda message, source=topic: self.on_action_status(message, source),
                10,
            )
            for topic in status_topics
        ]

        self.get_logger().info(
            "Waiting for a manipulation action abort; Ctrl-C writes a manual snapshot to "
            f"'{self.output_path}'",
        )

    def on_joint_state(self, message):
        self.latest_joint_state = message
        if self.capture_once and not self.capture_complete:
            self.capture("capture_once")

    def on_box_states(self, message):
        received_at = time.time()
        for state in message.boxes:
            self.latest_box_states[state.instance_id] = (state, received_at)
            tag_id = tag_id_from_instance(state.instance_id)
            if tag_id is not None:
                self.tag_ids.add(tag_id)

    def on_detections(self, message, source_topic):
        received_at = time.time()
        for detection in message.detections:
            self.tag_ids.add(detection.id)
            self.latest_detections[detection.id] = {
                "source_topic": source_topic,
                "frame_id": message.header.frame_id,
                "stamp": stamp_to_dict(message.header.stamp),
                "received_at_unix": received_at,
                "family": detection.family,
                "id": int(detection.id),
                "hamming": int(detection.hamming),
                "goodness": float(detection.goodness),
                "decision_margin": float(detection.decision_margin),
            }

    def on_action_status(self, message, source_topic):
        if self.capture_complete:
            return
        for status in message.status_list:
            if status.status == GoalStatus.STATUS_ABORTED:
                self.capture(
                    "action_aborted",
                    {
                        "topic": source_topic,
                        "goal_id": goal_id_to_hex(status.goal_info.goal_id),
                        "status": "STATUS_ABORTED",
                        "status_stamp": stamp_to_dict(status.goal_info.stamp),
                    },
                )
                return

    def lookup_transform(self, parent_frame, child_frame):
        try:
            return {"transform": transform_to_dict(
                self.tf_buffer.lookup_transform(parent_frame, child_frame, Time())
            )}
        except TransformException as error:
            return {
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "error": str(error),
            }

    def build_capture(self, reason, action_status=None):
        """Build a complete, YAML-serializable diagnostic snapshot."""
        captured_at = time.time()
        robot = {
            "joint_state_topic": self.joint_state_topic,
            "planning_frame": self.planning_frame,
        }
        if self.latest_joint_state is None:
            robot["joint_state_error"] = "no joint-state message received before capture"
        else:
            joint_state = joint_state_to_dict(self.latest_joint_state)
            joint_state["topic"] = self.joint_state_topic
            robot["joint_state"] = joint_state

        if self.robot_pose_parent_frame:
            robot["base_pose"] = self.lookup_transform(
                self.robot_pose_parent_frame, self.planning_frame
            )

        box_states = [
            box_state_to_dict(state, received_at)
            for _, (state, received_at) in sorted(self.latest_box_states.items())
        ]
        tags = []
        for tag_id in sorted(self.tag_ids):
            tag = {
                "tag_id": tag_id,
                "frame": f"{self.tag_frame_prefix}{tag_id}",
                "pose": self.lookup_transform(
                    self.planning_frame, f"{self.tag_frame_prefix}{tag_id}"
                ),
            }
            if tag_id in self.latest_detections:
                tag["latest_detection"] = self.latest_detections[tag_id]
            tags.append(tag)

        capture = {
            "schema_version": 1,
            "capture": {
                "reason": reason,
                "captured_at_unix": captured_at,
                "captured_at_utc": datetime.fromtimestamp(
                    captured_at, tz=timezone.utc
                ).isoformat(),
                "recorder_started_at_unix": self.capture_started_unix,
            },
            # Keep this top-level mapping compatible with fake_zmq_joint_states'
            # --initial-state-file input for replaying the joint configuration.
            "joint_positions": (
                robot.get("joint_state", {}).get("joint_positions", {})
            ),
            "robot": robot,
            "visible_box_states": box_states,
            "tags": tags,
        }
        if action_status is not None:
            capture["capture"]["action_status"] = action_status
        return capture

    def capture(self, reason, action_status=None):
        """Write one capture from the recorder's current cache."""
        if self.capture_complete:
            return
        capture = self.build_capture(reason, action_status)
        payload = yaml.safe_dump(capture, sort_keys=False, allow_unicode=True)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.output_path.open("x", encoding="utf-8") as stream:
                stream.write(payload)
        except OSError as error:
            self.get_logger().error(f"Could not write failure snapshot: {error}")
            self.capture_complete = True
            return
        self.capture_complete = True
        self.get_logger().warn(f"Wrote failure snapshot to '{self.output_path}'")


def main(argv=None):
    """Run until an action aborts, or write a manual snapshot on Ctrl-C."""
    argv = sys.argv if argv is None else argv
    arguments = parse_arguments(argv)
    rclpy.init(args=argv)
    node = None
    try:
        node = FailureSnapshotRecorder(arguments)
        while rclpy.ok() and not node.capture_complete:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        if node is not None and not node.capture_complete:
            node.capture("manual_interrupt")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
