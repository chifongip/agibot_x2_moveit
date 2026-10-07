"""Check launch selection and remaps without starting camera or robot nodes."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
import pytest


def module():
    path = Path(__file__).parents[1] / "launch" / "optimized_camera_apriltag.launch.py"
    spec = importlib.util.spec_from_file_location("optimized_camera_launch", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize(
    "rgbd,front,expected",
    [
        ("true", "true", 5),
        ("true", "false", 2),
        ("false", "true", 3),
        ("false", "false", 0),
    ],
)
def test_camera_selection(rgbd, front, expected):
    context = LaunchContext()
    description = module().generate_launch_description()
    context.launch_configurations.update(enable_rgbd=rgbd, enable_front_center=front)
    nodes = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            nodes.extend(action.execute(context))
        elif isinstance(action, Node):
            nodes.append(action)
    enabled = [n for n in nodes if n.condition is None or n.condition.evaluate(context)]
    assert len(enabled) == expected
    executables = [n.node_executable for n in enabled]
    assert set(executables) <= {
        "camera_preprocessor",
        "apriltag_node",
        "static_transform_publisher",
    }


def test_custom_relative_topics_and_static_tf_disable():
    context = LaunchContext()
    context.launch_configurations.update(
        rgbd_output_image="custom/image",
        rgbd_output_camera_info="custom/calibration",
        front_center_output_image="custom/image",
        front_center_output_camera_info="custom/calibration",
        publish_front_center_static_tf="false",
    )
    description = module().generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    detectors = [
        action.execute(context)[0]
        for action in description.entities
        if isinstance(action, OpaqueFunction)
    ]
    for detector, prefix in zip(detectors, ("", "/front_center_rectify")):
        remaps = detector._Node__remappings
        resolved = [
            (perform_substitutions(context, a), perform_substitutions(context, b))
            for a, b in remaps
        ]
        assert resolved == [
            ("image_rect", prefix + "/custom/image"),
            (prefix + "/custom/camera_info", prefix + "/custom/calibration"),
        ]
    assert not description.entities[-1].condition.evaluate(context)


def test_rgbd_input_defaults_use_color_camera_compressed_stream():
    context = LaunchContext()
    for action in module().generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert (
        context.launch_configurations["rgbd_compressed_image"]
        == "/camera/color/image_raw/compressed"
    )
    assert (
        context.launch_configurations["rgbd_camera_info"] == "/camera/color/camera_info"
    )
