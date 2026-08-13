import importlib.util
import os
from pathlib import Path

import pytest


os.environ.setdefault("ROS_LOG_DIR", "/tmp/agibot_x2_moveit_launch_test")

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchContext  # noqa: E402


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "real_robot.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("real_robot_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_description_constructs():
    description = load_launch_module().generate_launch_description()
    assert description.entities


def test_real_robot_defaults_preserve_state_delivery_headroom():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in source
    assert '"ros2_control_update_rate",\n                default_value="100"' in source


@pytest.mark.parametrize("source", ["none", "depth", "lidar", "both"])
def test_launch_setup_accepts_all_perception_modes(source):
    module = load_launch_module()
    config_path = Path(get_package_share_directory("agibot_x2_moveit_config"))
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "command_transport": "zmq",
            "zmq_endpoint": "tcp://*:8559",
            "leg_state_topic": "/aima/hal/joint/leg/state",
            "waist_state_topic": "/aima/hal/joint/waist/state",
            "arm_state_topic": "/x2_test/aima/hal/joint/arm/state",
            "head_state_topic": "/aima/hal/joint/head/state",
            "ros2_control_gains_file": str(
                config_path / "config" / "x2_ros2_control_gains.yaml"
            ),
            "use_rviz": "false",
            "perception_3d_source": source,
            "depth_image_topic": "/test/depth",
            "depth_camera_info_topic": "/test/depth_camera_info",
            "lidar_pointcloud_topic": "/test/points",
        }
    )

    assert len(module.launch_setup(context)) == 6
