#!/usr/bin/env python3
"""Mirror X2 ZMQ arm targets into fake AimDK joint-state topics."""

import argparse
import math
import sys
import time
from numbers import Real

import rclpy
import zmq
from aimdk_msgs.msg import JointState, JointStateArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


LEG_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]
WAIST_JOINT_NAMES = ["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"]
ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]
HEAD_JOINT_NAMES = ["head_yaw_joint", "head_pitch_joint"]
ALL_JOINT_NAMES = LEG_JOINT_NAMES + WAIST_JOINT_NAMES + ARM_JOINT_NAMES + HEAD_JOINT_NAMES

STATE_TOPICS = {
    "/aima/hal/joint/leg/state": LEG_JOINT_NAMES,
    "/aima/hal/joint/waist/state": WAIST_JOINT_NAMES,
    "/aima/hal/joint/arm/state": ARM_JOINT_NAMES,
    "/aima/hal/joint/head/state": HEAD_JOINT_NAMES,
}

LOCOMANIPULATION_POSITIONS = dict(
    zip(
        ALL_JOINT_NAMES,
        [
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
            0.0, 0.0, 0.0,
            0.35, 0.1, 0.0, -0.87, 0.0, 0.0, 0.0,
            0.35, -0.1, 0.0, -0.87, 0.0, 0.0, 0.0,
            0.0, 0.0,
        ],
    )
)


def validate_positions(message):
    """Validate one documented upper-body ZMQ message."""
    if not isinstance(message, dict) or "positions" not in message:
        raise ValueError("message must contain a 'positions' object")
    positions = message["positions"]
    if not isinstance(positions, dict) or not positions:
        raise ValueError("positions must be a non-empty object")
    unknown = sorted(set(positions) - set(ARM_JOINT_NAMES))
    if unknown:
        raise ValueError(f"unknown arm joints: {unknown}")

    decoded = {}
    for name, value in positions.items():
        if isinstance(value, bool) or not isinstance(value, Real):
            raise ValueError(f"position for {name} must be numeric")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"position for {name} must be finite")
        decoded[name] = value
    return decoded


class FakeZmqJointStates(Node):
    """Publish perfect X2 HAL state feedback for ZMQ arm commands."""

    def __init__(self, endpoint, publish_rate, initial_pose):
        super().__init__("fake_x2_zmq_joint_states")
        if publish_rate <= 0.0 or not math.isfinite(publish_rate):
            raise ValueError("publish_rate must be finite and positive")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publishers_ = {
            topic: self.create_publisher(JointStateArray, topic, sensor_qos)
            for topic in STATE_TOPICS
        }
        if initial_pose == "locomanipulation":
            self.positions_ = LOCOMANIPULATION_POSITIONS.copy()
        else:
            self.positions_ = {name: 0.0 for name in ALL_JOINT_NAMES}

        self.context_ = zmq.Context()
        self.socket_ = self.context_.socket(zmq.SUB)
        self.socket_.setsockopt(zmq.LINGER, 0)
        self.socket_.setsockopt(zmq.RCVHWM, 100)
        self.socket_.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket_.connect(endpoint)
        self.sequence_ = 0
        self.last_invalid_log_ = float("-inf")
        self.received_command_ = False
        self.create_timer(1.0 / publish_rate, self.update)
        self.get_logger().info(
            f"Publishing perfect X2 state feedback at {publish_rate:g} Hz; "
            f"subscribed to {endpoint}"
        )

    def close(self):
        """Release ZMQ resources immediately."""
        self.socket_.close(linger=0)
        self.context_.term()

    def receive_available(self):
        """Apply every queued message so partial updates retain prior values."""
        for _ in range(100):
            try:
                message = self.socket_.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            except (TypeError, ValueError, zmq.ZMQError) as exception:
                self.log_invalid(exception)
                continue

            try:
                positions = validate_positions(message)
            except ValueError as exception:
                self.log_invalid(exception)
                continue
            self.positions_.update(positions)
            if not self.received_command_:
                self.get_logger().info("Received the first valid ZMQ arm target")
                self.received_command_ = True

    def log_invalid(self, exception):
        """Throttle malformed-message warnings to one per second."""
        now = time.monotonic()
        if now - self.last_invalid_log_ >= 1.0:
            self.get_logger().warning(f"Rejected ZMQ target: {exception}")
            self.last_invalid_log_ = now

    def update(self):
        """Receive targets and publish one coherent state snapshot."""
        self.receive_available()
        stamp = self.get_clock().now().to_msg()
        for topic, names in STATE_TOPICS.items():
            message = JointStateArray()
            message.header.stamp = stamp
            message.header.meas_stamp = stamp
            message.header.sequence = self.sequence_
            message.joints = [
                JointState(
                    name=name,
                    position=self.positions_[name],
                    velocity=0.0,
                    effort=0.0,
                    error_code=0,
                )
                for name in names
            ]
            self.publishers_[topic].publish(message)
        self.sequence_ = (self.sequence_ + 1) % (2**32)


def parse_arguments(argv):
    """Parse utility arguments while preserving ROS-specific arguments."""
    parser = argparse.ArgumentParser(
        description="Publish perfect X2 HAL state feedback from MoveIt's ZMQ arm targets."
    )
    parser.add_argument(
        "--endpoint",
        default="tcp://127.0.0.1:8559",
        help="ZMQ PUB endpoint to connect to (default: %(default)s)",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=100.0,
        help="HAL state publication rate in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--initial-pose",
        choices=["locomanipulation", "zero"],
        default="locomanipulation",
        help="State used before the first ZMQ target (default: %(default)s)",
    )
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(argv=None):
    """Run the fake feedback node until interrupted."""
    argv = sys.argv if argv is None else argv
    arguments = parse_arguments(argv)
    rclpy.init(args=argv)
    node = FakeZmqJointStates(
        endpoint=arguments.endpoint,
        publish_rate=arguments.publish_rate,
        initial_pose=arguments.initial_pose,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
