import importlib.util
from pathlib import Path

import yaml
from sensor_msgs.msg import CameraInfo, Image

from agibot_x2_manipulation_image_test import (
    load_decompressor_module,
    load_raw_throttler_module,
)


CONFIG_FILE = Path(__file__).parents[1] / "config" / "box_manipulation.yaml"
RECORDED_JOINT_STATE_FILE = (
    Path(__file__).parents[1] / "config" / "recorded_planning_failure_joint_state.yaml"
)
RECORDED_TAG_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "recorded_planning_failure_dummy_apriltag.yaml"
)
LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "box_pick_place.launch.py"
RECORDED_LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "recorded_planning_failure.launch.py"
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location("box_pick_place_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_controls_perception_source_selection():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    # An exact node-scoped value takes precedence over the launch-generated
    # wildcard parameter file, so this selector must remain launch-owned.
    assert "perception_3d_source" not in config
    assert "allow_execution" not in config
    assert '"arm_state_topic": arm_state_topic' in LAUNCH_FILE.read_text(
        encoding="utf-8"
    )


def test_pose_to_pose_mode_is_selectable_and_closed_chain_remains_default():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    recorded_source = RECORDED_LAUNCH_FILE.read_text(encoding="utf-8")
    assert "motion_planning_mode" not in config
    assert '"motion_planning_mode",\n                default_value="closed_chain"' in launch_source
    assert 'choices=["closed_chain", "pose_to_pose"]' in launch_source
    assert '"motion_planning_mode": motion_planning_mode' in launch_source
    assert '"motion_planning_mode": motion_planning_mode' in recorded_source


def test_launch_defaults_preserve_state_delivery_headroom():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in source
    assert '"ros2_control_update_rate",\n                default_value="100"' in source


def test_launch_can_consume_existing_shared_state():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"start_state_bringup",' in source
    assert 'default_value="true"' in source
    assert '"start_state_bringup": start_state_bringup' in source


def test_launch_can_reuse_an_active_dual_arm_controller():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"spawn_dual_arm_controller",' in source
    assert 'default_value="true"' in source
    assert '"spawn_dual_arm_controller": spawn_dual_arm_controller' in source


def test_filtered_output_topics_match_moveit_configuration():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    assert config["depth_filtered_cloud_topic"] == "/x2/moveit/depth_filtered_cloud"
    assert config["lidar_filtered_cloud_topic"] == "/x2/moveit/lidar_filtered_cloud"


def test_execution_requires_fresh_settled_feedback():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    assert config["execution_settle_timeout"] > 0.0
    assert config["execution_joint_tolerance"] > 0.0
    assert config["execution_velocity_tolerance"] > 0.0
    assert config["execution_settle_samples"] >= 2


def test_recorded_failure_snapshot_is_complete_and_uses_full_box_pose():
    with RECORDED_JOINT_STATE_FILE.open(encoding="utf-8") as stream:
        joint_positions = yaml.safe_load(stream)["joint_positions"]
    with RECORDED_TAG_FILE.open(encoding="utf-8") as stream:
        tag_parameters = yaml.safe_load(stream)["dummy_apriltag"]["ros__parameters"]

    assert len(joint_positions) == 31
    assert "left_shoulder_pitch_joint" in joint_positions
    assert "right_ankle_roll_joint" in joint_positions
    assert tag_parameters["replay_box_dimensions"] == [0.15, 0.35, 0.32]
    assert len(tag_parameters["replay_box_pose"]) == 7
    assert tag_parameters["replay_box_pose"][4] != 0.0


def test_recorded_failure_launch_allows_isolated_execution_by_default():
    source = RECORDED_LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'allow_execution = LaunchConfiguration("allow_execution")' in source
    assert '"allow_execution",\n                default_value="true"' in source
    assert '"allow_execution": allow_execution' in source
    assert '"allow_execution": "false"' not in source
    assert 'f"/tmp/agibot_x2_recorded_replay_state_{uuid.uuid4().hex}"' in source
    assert '"manipulation_state_file": manipulation_state_file' in source


def test_coordinated_grasp_search_has_conservative_limits():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    localizer = document["box_localizer"]["ros__parameters"]
    config = document["pick_place_server"]["ros__parameters"]

    assert localizer["maximum_box_tilt"] <= 0.349066
    assert config["grasp_position_tolerance"] <= 0.020
    assert config["grasp_orientation_tolerance"] <= 0.139627
    assert config["maximum_grasp_candidates"] > 0
    assert config["ik_attempts_per_candidate"] >= 4
    assert config["maximum_planning_candidates"] > 0
    assert config["planning_time_per_candidate"] > 0.0
    assert config["pregrasp_planning_timeout"] >= 30.0
    assert config["maximum_retry_candidates"] == 3
    assert config["minimum_grasp_joint_margin"] >= 0.02
    assert config["closed_chain_position_tolerance"] == 0.010
    assert config["closed_chain_orientation_tolerance"] <= 0.052360
    assert config["closed_chain_ik_attempts"] >= 4
    assert config["closed_chain_beam_width"] == 8
    assert config["closed_chain_solutions_per_branch"] == 2
    assert config["closed_chain_projection_limit"] == 32
    assert config["closed_chain_validation_position_step"] <= 0.005
    assert config["closed_chain_validation_orientation_step"] <= 0.017454
    assert config["closed_chain_contact_position_error"] <= 0.002
    assert config["closed_chain_contact_orientation_error"] <= 0.017454
    assert config["carry_search_timeout"] == 8.0
    assert config["carry_search_z_lower"] >= 0.12
    assert config["carry_search_z_upper"] <= 0.03
    assert config["carry_search_x_range"] <= 0.05
    assert config["carry_search_y_range"] <= 0.03
    assert config["carry_search_orientation_tolerance"] <= 0.174534
    assert config["adaptive_place_position_tolerance"] == [0.015, 0.015, 0.005]
    assert config["adaptive_place_yaw_tolerance"] <= 0.087267
    assert (
        config["maximum_planning_candidates"]
        * config["planning_time_per_candidate"]
        < config["pregrasp_planning_timeout"]
    )


def test_x2_jpeg_end_marker_is_repaired():
    module = load_decompressor_module()
    truncated = b"\xff\xd8payload"
    assert module.repair_jpeg(truncated) == truncated + b"\xff\xd9"
    complete = truncated + b"\xff\xd9"
    assert module.repair_jpeg(complete) == complete


def test_raw_image_throttle_limits_frame_rate_and_preserves_camera_info():
    module = load_raw_throttler_module()
    limiter = module.FrameRateLimiter(1.0)
    camera_info = CameraInfo()
    camera_info.header.frame_id = "camera_calibration_frame"
    camera_info.k[0] = 500.0
    image = Image()
    image.header.frame_id = "camera_optical_frame"
    image.header.stamp.sec = 12
    image.header.stamp.nanosec = 34

    assert limiter.accept(0.0)
    assert not limiter.accept(0.999)
    assert limiter.accept(1.0)
    paired_camera_info = module.camera_info_for_image(camera_info, image)
    assert paired_camera_info.header == image.header
    assert paired_camera_info.k[0] == 500.0
    assert camera_info.header.frame_id == "camera_calibration_frame"


def test_apriltag_camera_subscriber_remaps_follow_image_namespace():
    module = load_launch_module()
    assert module.apriltag_remappings(
        "/x2/rgb_image_decompressed", "/robot/rgb_camera_info"
    ) == [
        ("/image_rect", "/x2/rgb_image_decompressed"),
        ("/x2/camera_info", "/robot/rgb_camera_info"),
    ]
    assert module.apriltag_remappings(
        "/aima/hal/sensor/rgbd_head_front/rgb_image",
        "/aima/hal/sensor/rgbd_head_front/rgb_camera_info",
    )[1][0] == "/aima/hal/sensor/rgbd_head_front/camera_info"


def test_apriltag_uses_throttled_raw_camera_topics_when_enabled():
    module = load_launch_module()
    assert module.apriltag_input_topics(
        True,
        "/camera/image_raw",
        "/camera/camera_info",
        "/x2/rgb_image_throttled",
        "/x2/rgb_image_throttled/camera_info",
    ) == (
        "/x2/rgb_image_throttled",
        "/x2/rgb_image_throttled/camera_info",
    )
    assert module.apriltag_input_topics(
        False,
        "/camera/image_raw",
        "/camera/camera_info",
        "/x2/rgb_image_throttled",
        "/x2/rgb_image_throttled/camera_info",
    ) == ("/camera/image_raw", "/camera/camera_info")
