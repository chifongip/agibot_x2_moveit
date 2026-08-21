#!/usr/bin/env python3
"""Rate-limit raw camera frames while preserving image/CameraInfo pairs."""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class FrameRateLimiter:
    """Accept at most one frame per configured monotonic-time interval."""

    def __init__(self, max_rate_hz: float) -> None:
        if not math.isfinite(max_rate_hz) or max_rate_hz <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        self.minimum_period = 1.0 / max_rate_hz
        self.last_publish_time = None

    def accept(self, now: float) -> bool:
        if (
            self.last_publish_time is not None
            and now - self.last_publish_time < self.minimum_period
        ):
            return False
        self.last_publish_time = now
        return True


def camera_info_for_image(camera_info: CameraInfo, image: Image) -> CameraInfo:
    """Copy calibration data and give it the selected image's header."""
    output = copy.deepcopy(camera_info)
    output.header = copy.deepcopy(image.header)
    return output


def input_qos(reliability: str) -> QoSProfile:
    if reliability not in ("best_effort", "reliable"):
        raise ValueError("input_reliability must be 'best_effort' or 'reliable'")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=DurabilityPolicy.VOLATILE,
    )


class RawImageThrottler(Node):
    def __init__(self) -> None:
        super().__init__("raw_image_throttler")
        input_image_topic = self.declare_parameter(
            "input_image_topic", "/aima/hal/sensor/rgbd_head_front/rgb_image"
        ).value
        input_camera_info_topic = self.declare_parameter(
            "input_camera_info_topic",
            "/aima/hal/sensor/rgbd_head_front/rgb_camera_info",
        ).value
        output_image_topic = self.declare_parameter(
            "output_image_topic", "/x2/rgb_image_throttled"
        ).value
        output_camera_info_topic = self.declare_parameter(
            "output_camera_info_topic", "/x2/rgb_image_throttled/camera_info"
        ).value
        max_rate_hz = float(self.declare_parameter("max_rate_hz", 10.0).value)
        reliability = str(
            self.declare_parameter("input_reliability", "best_effort").value
        ).lower()

        if not all(
            (
                input_image_topic,
                input_camera_info_topic,
                output_image_topic,
                output_camera_info_topic,
            )
        ):
            raise ValueError("raw image throttle topic names must not be empty")

        self.limiter = FrameRateLimiter(max_rate_hz)
        qos = input_qos(reliability)
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_publisher = self.create_publisher(Image, output_image_topic, output_qos)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, output_camera_info_topic, output_qos
        )
        self.image_subscription = self.create_subscription(
            Image, input_image_topic, self.on_image, qos
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, input_camera_info_topic, self.on_camera_info, qos
        )
        self.latest_camera_info = None
        self.received = 0
        self.published = 0
        self.rate_limited = 0
        self.missing_camera_info = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"Raw image throttle: {input_image_topic} -> {output_image_topic}, "
            f"maximum {max_rate_hz:g} Hz"
        )

    def on_camera_info(self, message: CameraInfo) -> None:
        self.latest_camera_info = message

    def on_image(self, message: Image) -> None:
        self.received += 1
        if self.latest_camera_info is None:
            self.missing_camera_info += 1
            return
        if not self.limiter.accept(time.monotonic()):
            self.rate_limited += 1
            return

        self.image_publisher.publish(message)
        self.camera_info_publisher.publish(
            camera_info_for_image(self.latest_camera_info, message)
        )
        self.published += 1

    def report(self) -> None:
        self.get_logger().info(
            f"frames: received={self.received}, published={self.published}, "
            f"rate_limited={self.rate_limited}, "
            f"missing_camera_info={self.missing_camera_info}"
        )


def main() -> None:
    rclpy.init()
    node = RawImageThrottler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
