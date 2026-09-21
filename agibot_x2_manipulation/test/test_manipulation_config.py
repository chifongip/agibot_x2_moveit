import importlib.util
from pathlib import Path
from xml.etree import ElementTree

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import yaml
from sensor_msgs.msg import CameraInfo, Image

from agibot_x2_manipulation_image_test import (
    load_decompressor_module,
    load_raw_throttler_module,
)


CONFIG_FILE = Path(__file__).parents[1] / "config" / "box_manipulation.yaml"
BOX_PROFILES_FILE = Path(__file__).parents[1] / "config" / "box_profiles.yaml"
RECORDED_JOINT_STATE_FILE = (
    Path(__file__).parents[1] / "config" / "recorded_planning_failure_joint_state.yaml"
)
RECORDED_TAG_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "recorded_planning_failure_dummy_apriltag.yaml"
)
LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "box_pick_place.launch.py"
TABLE_TAG_LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "rgb_head_front_center_apriltag.launch.py"
)
PACKAGE_FILE = Path(__file__).parents[1] / "package.xml"
RECORDED_LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "recorded_planning_failure.launch.py"
)
PLANNING_SCENE_MANAGER_FILE = (
    Path(__file__).parents[1]
    / "src"
    / "pick_place"
    / "planning_scene_manager.cpp"
)
MOVE_CARRY_POSE_ACTION_FILE = (
    Path(__file__).parents[2]
    / "agibot_x2_manipulation_msgs"
    / "action"
    / "MoveCarryPose.action"
)
POSTURE_SERVICE_FILE = (
    Path(__file__).parents[2]
    / "agibot_x2_manipulation_msgs"
    / "srv"
    / "SetLocomanipulationPosture.srv"
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


def test_independent_locomanipulation_posture_zmq_is_enabled_by_default():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    server_source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")
    controller_source = (
        Path(__file__).parents[1]
        / "src"
        / "pick_place"
        / "locomanipulation_posture_controller.cpp"
    ).read_text(encoding="utf-8")

    assert '"posture_zmq_enabled",\n                default_value="true"' in launch_source
    assert '"posture_zmq_enabled": ParameterValue(' in launch_source
    assert '"posture_zmq_endpoint": ParameterValue(' in launch_source
    assert '"leg_state_topic": leg_state_topic' in launch_source
    assert '"waist_state_topic": waist_state_topic' in launch_source
    assert "posture_zmq_endpoint" not in config
    assert "posture_default_height" not in config
    assert "posture_default_waist_yaw" not in config
    profile_header = (
        Path(__file__).parents[1]
        / "include"
        / "agibot_x2_manipulation"
        / "box_profile_registry.hpp"
    ).read_text(encoding="utf-8")
    assert "posture_height" not in profile_header
    assert "posture_waist_yaw" not in profile_header
    assert "create_service<SetLocomanipulationPosture>" in server_source
    assert '"/set_locomanipulation_posture"' in server_source
    assert "create_service<ClearLocomanipulationPostureTarget>" in server_source
    assert '"/clear_locomanipulation_posture_target"' in server_source
    assert '"/locomanipulation_posture_status"' in server_source
    assert "posture_controller_.deactivateTarget();" in server_source
    assert "posture changes require manipulation state EMPTY or HOLDING" in server_source
    assert "manipulation_state != ManipulationState::HOLDING" in server_source
    assert "posture execution is disabled: allow_execution is false" in server_source
    assert "prepareTaskPosture" not in server_source
    assert "prepareDefaultPosture" not in server_source
    assert "waitForPostureFeedback" not in server_source
    posture_service = POSTURE_SERVICE_FILE.read_text(encoding="utf-8")
    assert "float64 height" in posture_service
    assert "float64 waist_yaw" in posture_service
    assert "bool wait_for_settle" in posture_service
    assert "bool success" in posture_service
    posture_status = (
        Path(__file__).parents[2]
        / "agibot_x2_manipulation_msgs"
        / "msg"
        / "LocomanipulationPostureStatus.msg"
    ).read_text(encoding="utf-8")
    assert "bool execution_enabled" in posture_status
    assert "bool target_active" in posture_status
    assert '\\"height\\"' in controller_source
    assert '\\"waist_yaw\\"' in controller_source
    assert '"posture_zmq_endpoint", "tcp://*:8557"' in (
        Path(__file__).parents[1]
        / "src"
        / "pick_place"
        / "pick_place_config.cpp"
    ).read_text(encoding="utf-8")


def test_box_profiles_are_shared_by_localization_and_planning():
    with BOX_PROFILES_FILE.open(encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)["/**"]["ros__parameters"]

    profile = catalog["box_profiles"]["small_carton"]
    assert catalog["box_profiles_tag_frame_prefix"] == "tag"
    assert profile["tag_ids"] == [0]
    assert len(profile["dimensions"]) == 3
    assert len(profile["tag_to_box_center_pose"]) == 7
    assert "tag_to_box_offset" not in profile
    assert profile["pregrasp_distance"] > 0.0
    assert len(profile["carry_pose_a"]) == 7
    assert len(profile["carry_pose_b"]) == 7

    with CONFIG_FILE.open(encoding="utf-8") as stream:
        server_config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]
    assert server_config["visible_boxes_as_obstacles"] is True

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"box_profiles_file"' in launch_source
    assert "parameters=[params_file, box_profiles_file]" in launch_source
    assert "params_file,\n                    box_profiles_file," in launch_source


def test_profile_carry_pose_configuration_and_manual_transition_action_are_available():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]
    with BOX_PROFILES_FILE.open(encoding="utf-8") as stream:
        profiles = yaml.safe_load(stream)["/**"]["ros__parameters"]["box_profiles"]

    # These remain only for legacy deployments without box_profiles.
    assert len(config["carry_box_pose_a"]) == 7
    assert len(config["carry_box_pose_b"]) == 7
    assert all(len(profile["carry_pose_a"]) == 7 for profile in profiles.values())
    assert all(len(profile["carry_pose_b"]) == 7 for profile in profiles.values())
    action = MOVE_CARRY_POSE_ACTION_FILE.read_text(encoding="utf-8")
    assert "uint8 CARRY_A=0" in action
    assert "uint8 CARRY_B=1" in action
    assert "bool plan_only" in action


def test_local_scene_monitor_skips_octomap_without_3d_perception():
    source = PLANNING_SCENE_MANAGER_FILE.read_text(encoding="utf-8")

    assert "config_.perception_source != Perception3dSource::NONE" in source
    assert "startWorldGeometryMonitor(" in source
    assert "load_octomap_monitor);" in source


def test_adaptive_carry_collision_rejections_report_contact_pairs():
    scene_source = PLANNING_SCENE_MANAGER_FILE.read_text(encoding="utf-8")
    planner_source = (
        Path(__file__).parents[1]
        / "src"
        / "pick_place"
        / "dual_arm_motion_planner.cpp"
    ).read_text(encoding="utf-8")

    assert "request.contacts = true;" in scene_source
    assert "request.max_contacts_per_pair = 1;" in scene_source
    assert 'pair.first << " <-> " << pair.second' in scene_source
    assert "colliding_pairs" in planner_source
    assert "colliding_pairs=[" in planner_source


def test_grasped_box_may_contact_all_wrist_links_but_not_the_environment():
    scene_source = PLANNING_SCENE_MANAGER_FILE.read_text(encoding="utf-8")

    assert "std::vector<std::string> boxTouchLinks" in scene_source
    for link in (
        "left_wrist_yaw_link",
        "left_wrist_pitch_link",
        "left_wrist_roll_link",
        "right_wrist_yaw_link",
        "right_wrist_pitch_link",
        "right_wrist_roll_link",
    ):
        assert link in scene_source
    assert "object.touch_links = boxTouchLinks(config_);" in scene_source
    assert "config_.box_id, boxTouchLinks(config_), true" in scene_source


def test_late_visible_box_detections_do_not_interrupt_a_planned_task():
    server_source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")

    assert "A newly\n    // detected instance may be a late/stale tag observation" in server_source
    assert "if (actual.size() != expected.size())" not in server_source
    assert "visible box instance became stale before motion" in server_source
    assert "changed before motion:" in server_source


def test_detailed_planning_trace_file_is_automatic_and_launch_configurable():
    package_root = Path(__file__).parents[1]
    config_source = (
        package_root / "src" / "pick_place" / "pick_place_config.cpp"
    ).read_text(encoding="utf-8")
    planner_source = (
        package_root / "src" / "pick_place" / "dual_arm_motion_planner.cpp"
    ).read_text(encoding="utf-8")
    logger_source = (
        package_root / "src" / "pick_place" / "planning_trace_logger.cpp"
    ).read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'parameter<std::string>(node, "planning_log_file", "")' in config_source
    assert ('parameter<std::string>(\n    node, "planning_log_directory", '
            '"/tmp/agibot_x2_planning_traces")') in config_source
    assert "planning_log_file must be an absolute path when enabled" in config_source
    assert "planning_log_directory must be an absolute path when enabled" in config_source
    assert 'DeclareLaunchArgument(\n                "planning_log_file"' in launch_source
    assert 'DeclareLaunchArgument(\n                "planning_log_directory"' in launch_source
    assert '"planning_log_file": planning_log_file' in launch_source
    assert '"planning_log_directory": planning_log_directory' in launch_source
    assert 'filename << "planning-" << std::put_time' in logger_source
    assert '"-" << getpid() << ".jsonl"' in logger_source
    assert "adaptive_carry_endpoint_precheck" in planner_source
    assert "adaptive_carry_selected" in planner_source


def test_empty_operations_reconcile_detections_and_restart_cleanup_remains_available():
    scene_source = PLANNING_SCENE_MANAGER_FILE.read_text(encoding="utf-8")
    server_source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")

    assert "bool PlanningSceneManager::clearManagedBoxes" in scene_source
    assert "managed_box_id_prefix_" in scene_source
    assert "scene_interface_.getObjects()" in scene_source
    assert "scene_interface_.getAttachedObjects()" in scene_source
    assert "clearManagedBoxes(error)" in scene_source
    assert "clearSceneAfterEmptyOperation(task);" in server_source
    assert 'updateVisibleBoxScene("", false, true, visible_boxes, error)' in server_source
    assert (
        "planning_scene_.updateDetectionScene(observations, protected_ids, false, error)"
        in server_source
    )
    assert (
        "planning_scene_.updateDetectionScene(observations, {}, true, error)"
        in server_source
    )


def test_post_place_separates_coordinated_retreat_from_named_target_planning():
    server_source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")
    assert "motion_planner_.buildRetreat(" in server_source
    assert "post_place_planner_->plan(retreat_end, {}, scene, false" in server_source
    assert "validatePostPlaceSegment(" in server_source
    assert "postPlaceRetreatTarget" not in server_source


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
    assert '"initial_arm_command_mode",\n                default_value="measured"' in source
    assert '"initial_arm_command_mode": initial_arm_command_mode' in source


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


def test_recovery_service_does_not_starve_joint_state_callbacks():
    source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")

    assert "recovery_callback_group_ = node_->create_callback_group(" in source
    assert "rmw_qos_profile_services_default, recovery_callback_group_" in source
    assert "rclcpp::ExecutorOptions(), 2" in source


def test_place_endpoint_rejection_reports_requested_pose():
    source = (
        Path(__file__).parents[1]
        / "src"
        / "pick_place"
        / "dual_arm_motion_planner.cpp"
    ).read_text(encoding="utf-8")

    assert "requested_place_pose(frame=" in source
    assert "formatPose(requested_pose)" in source


def test_carry_routes_enforce_the_configured_joint_margin():
    source = (
        Path(__file__).parents[1]
        / "src"
        / "pick_place"
        / "dual_arm_motion_planner.cpp"
    ).read_text(encoding="utf-8")

    assert "validateMinimumJointMargin" in source
    assert "config_.minimum_carry_joint_margin" in source
    assert "joint_margin_rejected" in source
    assert "ClosedChainFailure::JOINT_MARGIN" in source


def test_filtered_output_topics_match_moveit_configuration():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    assert config["depth_filtered_cloud_topic"] == "/x2/moveit/depth_filtered_cloud"
    assert config["lidar_filtered_cloud_topic"] == "/x2/moveit/lidar_filtered_cloud"


def test_tag9_derives_the_default_table_place_pose():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["pick_place_server"]["ros__parameters"]

    assert config["use_tag_derived_place_pose"] is True
    assert config["table_tag_frame"] == "tag9"
    assert config["table_tag_to_tabletop_center"] == [0.0, -0.55, 0.15]
    assert config["table_tag_place_offset"] == [0.0, 0.05]
    assert config["table_collision_enabled"] is True
    assert config["table_collision_id"] == "work_table"
    assert config["table_dimensions"] == [0.6, 0.4, 0.6]
    assert "tag_to_box_offset" not in config
    assert config["maximum_table_tag_pose_age"] > 0.0
    assert config["table_tag_detections_topic"] == "/front_center_rectify/detections"
    assert config["table_tag_id"] == 9
    assert config["table_tag_stable_sample_count"] == 3
    assert config["table_tag_maximum_position_spread"] == 0.005
    assert config["table_tag_maximum_angular_spread"] == 0.0523598776
    assert config["table_tag_maximum_sample_gap"] == 2.5


def test_table_collision_is_published_as_a_latched_visualization_marker():
    source = PLANNING_SCENE_MANAGER_FILE.read_text(encoding="utf-8")
    server_source = (
        Path(__file__).parents[1] / "src" / "pick_place_server.cpp"
    ).read_text(encoding="utf-8")

    assert '"/table_markers", rclcpp::QoS(1).transient_local()' in source
    assert "void PlanningSceneManager::publishTableMarker" in source
    assert 'marker.ns = "collision_table"' in source
    assert "marker.type = visualization_msgs::msg::Marker::CUBE" in source
    assert "marker.pose = toPoseMsg(pose)" in source
    assert "marker.scale.x = config_.table_dimensions.length" in source
    assert "marker.scale.y = config_.table_dimensions.width" in source
    assert "marker.scale.z = config_.table_dimensions.height" in source
    assert "publishTrackedTableMarker(tag_pose);" in server_source
    assert "tablePoseFromVerticalTag(" in server_source
    assert "tag_pose.header.stamp" in server_source


def test_default_launch_starts_the_table_tag_detector_at_one_hz():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    table_tag_source = TABLE_TAG_LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"start_table_tag_detector",\n                default_value="true"' in source
    assert '"table_tag_detector_max_rate_hz",\n                default_value="1.0"' in source
    assert '"rgb_head_front_center_apriltag.launch.py"' in source
    assert "table_tag_detector_enabled = PythonExpression" in source
    assert "condition=IfCondition(table_tag_detector_enabled)" in source
    assert '"max_rate_hz": table_tag_detector_max_rate_hz' in source
    assert '"disable_table_collision": ParameterValue(' in source
    assert "name='max_rate_hz', default_value='1.0'" in table_tag_source


def test_table_tag_pipeline_resizes_before_rectification_with_scaled_camera_info():
    source = TABLE_TAG_LAUNCH_FILE.read_text(encoding="utf-8")

    assert source.index("plugin='image_proc::ResizeNode'") < source.index(
        "plugin='image_proc::RectifyNode'"
    )
    assert "'use_scale': False" in source
    assert "'width': ParameterValue(resize_width, value_type=int)" in source
    assert "'height': ParameterValue(resize_height, value_type=int)" in source
    assert "('image/camera_info', 'raw_camera_info')" in source
    assert "('resize/camera_info', 'camera_info')" in source
    assert "'output_camera_info_topic': 'raw_camera_info'" in source
    assert "name='resize_width', default_value='640'" in source
    assert "name='resize_height', default_value='480'" in source


def test_dummy_mode_and_explicit_disable_stop_the_table_tag_detector(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    module = load_launch_module()
    launch_description = module.generate_launch_description()
    table_tag_include = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription) and entity.condition is not None
    )
    context = LaunchContext()
    context.launch_configurations["start_table_tag_detector"] = "true"
    context.launch_configurations["use_dummy_apriltag"] = "false"
    assert table_tag_include.condition.evaluate(context)

    context.launch_configurations["use_dummy_apriltag"] = "true"
    assert not table_tag_include.condition.evaluate(context)

    context.launch_configurations["start_table_tag_detector"] = "false"
    assert not table_tag_include.condition.evaluate(context)


def test_table_tag_detector_runtime_dependencies_are_declared():
    root = ElementTree.parse(PACKAGE_FILE).getroot()
    dependencies = {entry.text for entry in root if entry.text}

    assert {
        "compressed_image_transport",
        "image_proc",
        "image_transport",
        "rclcpp_components",
        "topic_tools",
    } <= dependencies


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


def test_table_placement_is_independent_of_pickup_tag_calibration():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    localizer = document["box_localizer"]["ros__parameters"]
    server = document["pick_place_server"]["ros__parameters"]
    assert "tag_to_box_yaw" in localizer
    assert "tag_to_box_offset" in localizer
    assert "tag_to_box_yaw" not in server
    assert "tag_to_box_offset" not in server


def test_coordinated_grasp_search_has_conservative_limits():
    with CONFIG_FILE.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    localizer = document["box_localizer"]["ros__parameters"]
    config = document["pick_place_server"]["ros__parameters"]

    assert localizer["maximum_box_tilt"] <= 0.349066
    assert config["grasp_position_tolerance"] <= 0.1
    assert config["grasp_orientation_tolerance"] <= 0.174534
    assert config["maximum_grasp_candidates"] > 0
    assert config["ik_attempts_per_candidate"] >= 4
    assert config["maximum_planning_candidates"] > 0
    assert config["planning_time_per_candidate"] > 0.0
    assert config["pregrasp_planning_timeout"] >= 30.0
    assert config["maximum_retry_candidates"] == 3
    assert config["minimum_grasp_joint_margin"] == 0.0
    assert config["minimum_carry_joint_margin"] == config["minimum_grasp_joint_margin"]
    assert config["closed_chain_position_tolerance"] == 0.010
    assert config["closed_chain_orientation_tolerance"] <= 0.052360
    assert config["closed_chain_ik_attempts"] >= 4
    assert config["closed_chain_beam_width"] == 8
    assert config["closed_chain_solutions_per_branch"] == 2
    assert config["closed_chain_projection_limit"] == 32
    assert config["closed_chain_validation_position_step"] <= 0.005
    assert config["closed_chain_validation_orientation_step"] <= 0.017454
    assert config["closed_chain_contact_position_error"] <= 0.1
    assert config["closed_chain_contact_orientation_error"] <= 0.174534
    assert config["carry_search_timeout"] == 80.0
    assert config["maximum_carry_candidates"] == 270
    assert config["carry_search_z_lower"] >= 0.02
    assert config["carry_search_z_upper"] <= 0.05
    assert config["carry_search_x_range"] <= 0.05
    assert config["carry_search_y_range"] <= 0.05
    assert config["carry_search_orientation_tolerance"] <= 0.174534
    assert config["adaptive_place_position_tolerance"] == [0.05, 0.05, 0.02]
    assert config["adaptive_place_yaw_tolerance"] <= 0.174534
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
