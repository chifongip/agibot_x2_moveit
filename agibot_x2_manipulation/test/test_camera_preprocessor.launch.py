"""Exercise the actual preprocessor without robot interfaces."""

import time
import os
import unittest

import cv2
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
def generate_test_description():
    process = launch_ros.actions.Node(
        package="agibot_x2_manipulation",
        executable="camera_preprocessor",
        namespace="camera_preprocessor_test",
        parameters=[
            {
                "input_image_topic": "compressed",
                "input_camera_info_topic": "info",
                "output_image_topic": "rect",
                "output_camera_info_topic": "rect_info",
                "width": 320,
                "height": 240,
                "max_rate_hz": 5.0,
            }
        ],
    )
    config = os.path.join(
        get_package_share_directory("agibot_x2_manipulation"), "config", "apriltag.yaml"
    )
    detectors = []
    for name, image, info in [
        (
            "optimized",
            "/camera_preprocessor_test/rect",
            "/camera_preprocessor_test/rect_info",
        ),
        (
            "reference",
            "/camera_preprocessor_test/reference/image",
            "/camera_preprocessor_test/reference/camera_info",
        ),
    ]:
        detectors.append(
            launch_ros.actions.Node(
                package="apriltag_ros",
                executable="apriltag_node",
                namespace=f"camera_preprocessor_test/{name}",
                parameters=[config],
                remappings=[
                    ("image_rect", image),
                    (image.rsplit("/", 1)[0] + "/camera_info", info),
                    ("/tf", f"/camera_preprocessor_test/{name}/tf"),
                ],
            )
        )
    return launch.LaunchDescription(
        [process, *detectors, launch_testing.actions.ReadyToTest()]
    ), {"process": process}


class TestCameraPreprocessor(unittest.TestCase):
    def test_stream(self):
        rclpy.init()
        node = rclpy.create_node("camera_preprocessor_tester")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        outputs, infos, arrivals = [], {}, []
        node.create_subscription(
            Image,
            "/camera_preprocessor_test/rect",
            lambda msg: (outputs.append(msg), arrivals.append(time.monotonic())),
            qos,
        )
        node.create_subscription(
            CameraInfo,
            "/camera_preprocessor_test/rect_info",
            lambda msg: infos.update({msg.header.stamp.nanosec: msg}),
            qos,
        )
        publisher = node.create_publisher(
            CompressedImage, "/camera_preprocessor_test/compressed", 1
        )
        info_pub = node.create_publisher(
            CameraInfo, "/camera_preprocessor_test/info", qos
        )
        raw_pub = node.create_publisher(
            Image, "/camera_preprocessor_test/reference/image", qos
        )
        raw_info_pub = node.create_publisher(
            CameraInfo, "/camera_preprocessor_test/reference/camera_info", qos
        )
        detections = {"optimized": [], "reference": []}
        transforms = {"optimized": [], "reference": []}
        for name in detections:
            node.create_subscription(
                AprilTagDetectionArray,
                f"/camera_preprocessor_test/{name}/detections",
                lambda msg, key=name: detections[key].extend(msg.detections),
                qos,
            )
            node.create_subscription(
                TFMessage,
                f"/camera_preprocessor_test/{name}/tf",
                lambda msg, key=name: transforms[key].extend(msg.transforms),
                qos,
            )
        info = CameraInfo()
        info.header.frame_id = "camera"
        info.width, info.height = 640, 480
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 8
        info.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        pixels = np.full((480, 640), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 0, 200)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 0, 200)
        pixels[140:340, 220:420] = marker
        encoded = cv2.imencode(".jpg", pixels)[1].tobytes()
        raw = Image()
        raw.header.frame_id = "camera"
        raw.height, raw.width, raw.step, raw.encoding = 480, 640, 640, "mono8"
        raw.data = cv2.imdecode(
            np.frombuffer(encoded, np.uint8), cv2.IMREAD_GRAYSCALE
        ).tobytes()
        frame = CompressedImage()
        frame.header.frame_id = "camera"
        frame.format = "jpeg"
        frame.data = encoded
        try:
            deadline = time.monotonic() + 10
            while (
                publisher.get_subscription_count() == 0 and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(publisher.get_subscription_count(), 0)
            # Missing calibration must not produce an uncalibrated output.
            publisher.publish(frame)
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(outputs, [])
            # Recover from malformed JPEG and mismatched dimensions.
            info_pub.publish(info)
            frame.data = b"bad jpeg"
            publisher.publish(frame)
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(outputs, [])
            frame.data = encoded
            end = time.monotonic() + 2
            count = 0
            while time.monotonic() < end:
                count += 1
                frame.header.stamp.sec = 1
                frame.header.stamp.nanosec = count
                info_pub.publish(info)
                publisher.publish(frame)
                raw.header = frame.header
                info.header = frame.header
                raw_info_pub.publish(info)
                raw_pub.publish(raw)
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.005)
            end = time.monotonic() + 0.4
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertGreaterEqual(len(outputs), 5)
            self.assertLessEqual(len(outputs), 12)
            self.assertTrue(all(b - a > 0.15 for a, b in zip(arrivals, arrivals[1:])))
            self.assertGreater(outputs[-1].header.stamp.nanosec, count - 5)
            for image in outputs:
                paired = infos[image.header.stamp.nanosec]
                self.assertEqual(image.header, paired.header)
                self.assertEqual(
                    (image.width, image.height, image.encoding), (320, 240, "mono8")
                )
                self.assertEqual(len(image.data), 320 * 240)
                self.assertEqual(paired.p[0], 200.0)
                self.assertEqual(paired.p[2], 160.0)
            for name in detections:
                self.assertTrue(any(item.id == 0 for item in detections[name]))
                self.assertTrue(
                    any(item.child_frame_id == "tag0" for item in transforms[name])
                )
            poses = {
                name: [
                    t.transform.translation
                    for t in transforms[name]
                    if t.child_frame_id == "tag0"
                ][-1]
                for name in transforms
            }
            for axis in ("x", "y", "z"):
                self.assertAlmostEqual(
                    getattr(poses["optimized"], axis),
                    getattr(poses["reference"], axis),
                    delta=0.002,
                )
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit(self, proc_info, process):
        launch_testing.asserts.assertExitCodes(proc_info, process=process)
