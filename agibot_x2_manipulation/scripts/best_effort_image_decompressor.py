#!/usr/bin/env python3
"""Decode the newest CompressedImage without blocking its DDS reader."""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


JPEG_END_MARKER = b"\xff\xd9"


def repair_jpeg(payload: bytes) -> bytes:
    """Append the JPEG end marker omitted by some X2 camera payloads."""
    if payload.startswith(b"\xff\xd8") and not payload.endswith(JPEG_END_MARKER):
        return payload + JPEG_END_MARKER
    return payload


class BestEffortImageDecompressor(Node):
    def __init__(self) -> None:
        super().__init__("best_effort_image_decompressor")
        input_topic = self.declare_parameter(
            "input_topic",
            "/aima/hal/sensor/rgbd_head_front/rgb_image/compressed",
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/x2/rgb_image_decompressed"
        ).value
        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 10.0).value)
        input_reliability = str(
            self.declare_parameter("input_reliability", "reliable").value
        ).lower()
        if not math.isfinite(self.max_rate_hz) or self.max_rate_hz <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        if input_reliability not in ("best_effort", "reliable"):
            raise ValueError(
                "input_reliability must be 'best_effort' or 'reliable'"
            )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if input_reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, output_topic, output_qos)
        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self.on_image, input_qos
        )
        self.minimum_period = 1.0 / self.max_rate_hz
        self.received = 0
        self.decoded = 0
        self.dropped = 0
        self.errors = 0
        self.condition = threading.Condition()
        self.latest_message = None
        self.stopping = False
        self.worker = threading.Thread(target=self.decode_loop, daemon=True)
        self.worker.start()
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"Latest-frame JPEG decoder: {input_topic} -> {output_topic}, "
            f"{input_reliability} input, maximum {self.max_rate_hz:g} Hz"
        )

    def on_image(self, message: CompressedImage) -> None:
        with self.condition:
            self.received += 1
            if self.latest_message is not None:
                self.dropped += 1
            self.latest_message = message
            self.condition.notify()

    def decode_loop(self) -> None:
        next_decode_time = time.monotonic()
        while True:
            with self.condition:
                while self.latest_message is None and not self.stopping:
                    self.condition.wait()
                if self.stopping:
                    return
                message = self.latest_message
                self.latest_message = None

            delay = next_decode_time - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            self.decode(message)
            next_decode_time = max(
                next_decode_time + self.minimum_period, time.monotonic()
            )

    def decode(self, message: CompressedImage) -> None:
        payload = repair_jpeg(bytes(message.data))
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.errors += 1
            self.get_logger().warning(
                "Could not decode compressed camera frame", throttle_duration_sec=5.0
            )
            return

        output = Image()
        output.header = message.header
        output.height, output.width = image.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = image.tobytes()
        if not rclpy.ok():
            return
        try:
            self.publisher.publish(output)
        except RuntimeError:
            # SIGINT can invalidate the ROS context while the worker is
            # finishing an OpenCV decode.
            if rclpy.ok():
                raise
            return
        self.decoded += 1

    def stop(self) -> None:
        with self.condition:
            self.stopping = True
            self.condition.notify()
        self.worker.join(timeout=2.0)

    def report(self) -> None:
        self.get_logger().info(
            f"frames: received={self.received}, decoded={self.decoded}, "
            f"rate_limited={self.dropped}, errors={self.errors}"
        )


def main() -> None:
    rclpy.init()
    node = BestEffortImageDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
