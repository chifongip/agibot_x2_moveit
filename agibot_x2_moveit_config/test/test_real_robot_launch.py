import importlib.util
import os
from pathlib import Path

import pytest


os.environ.setdefault("ROS_LOG_DIR", "/tmp/agibot_x2_moveit_launch_test")

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchContext  # noqa: E402


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "real_robot.launch.py"
DEMO_LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "demo.launch.py"
RSP_LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "rsp.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("real_robot_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_demo_launch_module():
    spec = importlib.util.spec_from_file_location("demo_launch", DEMO_LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_rsp_launch_module():
    spec = importlib.util.spec_from_file_location("rsp_launch", RSP_LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_description_constructs():
    description = load_launch_module().generate_launch_description()
    assert description.entities


def test_demo_launch_uses_shared_fake_state_bringup():
    description = load_demo_launch_module().generate_launch_description()
    source = DEMO_LAUNCH_FILE.read_text(encoding="utf-8")

    assert description.entities
    assert "state_publisher.launch.py" in source
    assert '"use_fake_hardware": "true"' in source
    assert "static_virtual_joint_tfs.launch.py" in source
    assert "move_group.launch.py" in source
    assert "moveit_rviz.launch.py" in source


def test_rsp_launch_delegates_to_passive_shared_publisher():
    description = load_rsp_launch_module().generate_launch_description()
    source = RSP_LAUNCH_FILE.read_text(encoding="utf-8")

    assert description.entities
    assert '"rsp.launch.py"' in source
    assert "state_publisher.launch.py" not in source
    assert 'package="robot_state_publisher"' not in source


def test_real_robot_defaults_preserve_state_delivery_headroom():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in source
    assert '"ros2_control_update_rate",\n                default_value="100"' in source


@pytest.mark.parametrize("source", ["none", "depth", "lidar", "both"])
def test_launch_setup_accepts_all_perception_modes(source):
    module = load_launch_module()
    bringup_path = Path(get_package_share_directory("x2_bringup"))
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
                bringup_path / "config" / "x2_ros2_control_gains.yaml"
            ),
            "use_rviz": "false",
            "start_state_bringup": "true",
            "perception_3d_source": source,
            "depth_image_topic": "/test/depth",
            "depth_camera_info_topic": "/test/depth_camera_info",
            "lidar_pointcloud_topic": "/test/points",
        }
    )

    assert len(module.launch_setup(context)) == 4


def test_real_robot_can_consume_existing_shared_state():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"start_state_bringup",' in source
    assert 'default_value="true"' in source
    assert "condition=IfCondition(start_state_bringup)" in source
