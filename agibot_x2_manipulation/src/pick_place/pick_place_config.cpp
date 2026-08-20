#include "pick_place/pick_place_config.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

std::string defaultStateFile()
{
  const char * ros_home = std::getenv("ROS_HOME");
  if (ros_home && ros_home[0] != '\0') {
    return std::string(ros_home) + "/agibot_x2_manipulation_state";
  }
  const char * user_home = std::getenv("HOME");
  return user_home && user_home[0] != '\0' ?
         std::string(user_home) + "/.ros/agibot_x2_manipulation_state" :
         "/tmp/agibot_x2_manipulation_state";
}

Eigen::Isometry3d poseFromParameter(const std::vector<double> & values)
{
  Eigen::Quaterniond rotation(values[6], values[3], values[4], values[5]);
  if (rotation.norm() < 1e-9) {
    throw std::invalid_argument("pose quaternion has zero length");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

}  // namespace

MotionPlanningMode parseMotionPlanningMode(const std::string & value)
{
  if (value == "closed_chain") {
    return MotionPlanningMode::CLOSED_CHAIN;
  }
  if (value == "pose_to_pose") {
    return MotionPlanningMode::POSE_TO_POSE;
  }
  throw std::runtime_error(
          "motion_planning_mode must be 'closed_chain' or 'pose_to_pose'");
}

const char * motionPlanningModeName(MotionPlanningMode mode)
{
  return mode == MotionPlanningMode::POSE_TO_POSE ? "pose_to_pose" : "closed_chain";
}

PickPlaceConfig loadPickPlaceConfig(const rclcpp::Node::SharedPtr & node)
{
  PickPlaceConfig config;
  config.planning_frame = parameter<std::string>(node, "planning_frame", "base_link");
  config.box_pose_topic = parameter<std::string>(node, "box_pose_topic", "/box_pose");
  config.planning_group = parameter<std::string>(node, "planning_group", "dual_arm");
  config.left_group_name = parameter<std::string>(node, "left_group", "left_arm");
  config.right_group_name = parameter<std::string>(node, "right_group", "right_arm");
  config.left_tcp = parameter<std::string>(node, "left_tcp", "left_hand_tcp_link");
  config.right_tcp = parameter<std::string>(node, "right_tcp", "right_hand_tcp_link");
  config.motion_planning_mode_name = parameter<std::string>(
    node, "motion_planning_mode", "closed_chain");
  config.motion_planning_mode = parseMotionPlanningMode(config.motion_planning_mode_name);
  config.box_id = parameter<std::string>(node, "box_id", "grasp_box");
  const auto dimensions = parameter<std::vector<double>>(
    node, "box_dimensions", {0.30, 0.20, 0.15});
  if (dimensions.size() != 3U) {
    throw std::runtime_error("box_dimensions must contain [length, width, height]");
  }
  config.dimensions = {dimensions[0], dimensions[1], dimensions[2]};
  config.pregrasp_distance = parameter<double>(node, "pregrasp_distance", 0.08);
  config.contact_height_offset = parameter<double>(node, "contact_height_offset", 0.0);
  config.lift_height = parameter<double>(node, "lift_height", 0.05);
  config.cartesian_step = parameter<double>(node, "cartesian_step", 0.01);
  config.max_pose_age = parameter<double>(node, "maximum_box_pose_age", 0.50);
  config.ik_timeout = parameter<double>(node, "ik_timeout", 0.05);
  config.grasp_position_tolerance = parameter<double>(node, "grasp_position_tolerance", 0.015);
  config.grasp_orientation_tolerance = parameter<double>(
    node, "grasp_orientation_tolerance", 0.0872664626);
  config.pregrasp_distance_tolerance = parameter<double>(
    node, "pregrasp_distance_tolerance", 0.015);
  config.alternate_face_alignment_tolerance = parameter<double>(
    node, "alternate_face_alignment_tolerance", 0.261799388);
  config.maximum_grasp_candidates = parameter<int>(node, "maximum_grasp_candidates", 64);
  config.grasp_search_timeout = parameter<double>(node, "grasp_search_timeout", 2.0);
  config.ik_attempts_per_candidate = parameter<int>(node, "ik_attempts_per_candidate", 4);
  config.maximum_planning_candidates = parameter<int>(node, "maximum_planning_candidates", 5);
  config.planning_time_per_candidate = parameter<double>(
    node, "planning_time_per_candidate", 2.0);
  config.pregrasp_planning_timeout = parameter<double>(node, "pregrasp_planning_timeout", 30.0);
  config.maximum_retry_candidates = parameter<int>(node, "maximum_retry_candidates", 3);
  config.minimum_grasp_joint_margin = parameter<double>(node, "minimum_grasp_joint_margin", 0.02);
  config.closed_chain_position_tolerance = parameter<double>(
    node, "closed_chain_position_tolerance", 0.010);
  config.closed_chain_orientation_tolerance = parameter<double>(
    node, "closed_chain_orientation_tolerance", 0.0523598776);
  config.closed_chain_position_step = parameter<double>(node, "closed_chain_position_step", 0.005);
  config.closed_chain_orientation_step = parameter<double>(
    node, "closed_chain_orientation_step", 0.0261799388);
  config.closed_chain_ik_attempts = parameter<int>(node, "closed_chain_ik_attempts", 4);
  config.closed_chain_ik_timeout = parameter<double>(node, "closed_chain_ik_timeout", 0.10);
  config.closed_chain_beam_width = parameter<int>(node, "closed_chain_beam_width", 8);
  config.closed_chain_solutions_per_branch = parameter<int>(
    node, "closed_chain_solutions_per_branch", 2);
  config.closed_chain_projection_limit = parameter<int>(node, "closed_chain_projection_limit", 32);
  config.closed_chain_validation_position_step = parameter<double>(
    node, "closed_chain_validation_position_step", 0.005);
  config.closed_chain_validation_orientation_step = parameter<double>(
    node, "closed_chain_validation_orientation_step", 0.0174533);
  config.closed_chain_contact_position_error = parameter<double>(
    node, "closed_chain_contact_position_error", 0.002);
  config.closed_chain_contact_orientation_error = parameter<double>(
    node, "closed_chain_contact_orientation_error", 0.0174533);
  config.carry_search_timeout = parameter<double>(node, "carry_search_timeout", 8.0);
  config.maximum_carry_candidates = parameter<int>(node, "maximum_carry_candidates", 96);
  config.carry_search_x_range = parameter<double>(node, "carry_search_x_range", 0.05);
  config.carry_search_y_range = parameter<double>(node, "carry_search_y_range", 0.03);
  config.carry_search_z_lower = parameter<double>(node, "carry_search_z_lower", 0.12);
  config.carry_search_z_upper = parameter<double>(node, "carry_search_z_upper", 0.03);
  config.carry_search_orientation_tolerance = parameter<double>(
    node, "carry_search_orientation_tolerance", 0.1745329252);
  const auto place_tolerance = parameter<std::vector<double>>(
    node, "adaptive_place_position_tolerance", {0.015, 0.015, 0.005});
  if (place_tolerance.size() != 3U) {
    throw std::runtime_error("adaptive_place_position_tolerance must contain [x, y, z]");
  }
  config.adaptive_place_position_tolerance = Eigen::Vector3d(
    place_tolerance[0], place_tolerance[1], place_tolerance[2]);
  config.adaptive_place_yaw_tolerance = parameter<double>(
    node, "adaptive_place_yaw_tolerance", 0.0872664626);
  config.max_joint_step = parameter<double>(node, "maximum_joint_step", 0.35);
  config.allow_execution = parameter<bool>(node, "allow_execution", false);
  config.velocity_scaling = parameter<double>(node, "velocity_scaling", 0.10);
  config.acceleration_scaling = parameter<double>(node, "acceleration_scaling", 0.10);
  const auto carry_pose = parameter<std::vector<double>>(
    node, "carry_box_pose", {0.35, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0});
  if (carry_pose.size() != 7U) {
    throw std::runtime_error("carry_box_pose must contain [x, y, z, qx, qy, qz, qw]");
  }
  config.carry_pose = poseFromParameter(carry_pose);
  config.recovery_position_tolerance = parameter<double>(
    node, "recovery_position_tolerance", 0.04);
  config.recovery_angular_tolerance = parameter<double>(
    node, "recovery_angular_tolerance", 0.1745329252);
  config.state_file = parameter<std::string>(node, "state_file", defaultStateFile());
  config.initial_state = parameter<std::string>(node, "initial_state", "empty");
  config.post_place_named_target = parameter<std::string>(node, "post_place_named_target", "zero");
  config.reset_named_target = parameter<std::string>(node, "reset_named_target", "zero");
  config.reset_preemption_timeout = parameter<double>(node, "reset_preemption_timeout", 15.0);
  config.reset_state_timeout = parameter<double>(node, "reset_state_timeout", 2.0);
  config.reset_joint_tolerance = parameter<double>(node, "reset_joint_tolerance", 0.02);
  config.execution_settle_timeout = parameter<double>(
    node, "execution_settle_timeout", config.reset_state_timeout);
  config.execution_joint_tolerance = parameter<double>(
    node, "execution_joint_tolerance", config.reset_joint_tolerance);
  config.execution_velocity_tolerance = parameter<double>(
    node, "execution_velocity_tolerance", 0.01);
  config.execution_settle_samples = parameter<int>(node, "execution_settle_samples", 3);
  config.arm_state_topic = parameter<std::string>(
    node, "arm_state_topic", "/aima/hal/joint/arm/state");
  if (config.reset_preemption_timeout <= 0.0 || config.reset_state_timeout <= 0.0 ||
    config.reset_joint_tolerance < 0.0 || config.execution_settle_timeout <= 0.0 ||
    config.execution_joint_tolerance < 0.0 || config.execution_velocity_tolerance < 0.0 ||
    config.execution_settle_samples < 1 || config.arm_state_topic.empty())
  {
    throw std::runtime_error(
            "reset/execution timeouts must be positive; tolerances must be nonnegative; "
            "arm_state_topic must not be empty");
  }
  if (config.initial_state != "empty" && config.initial_state != "unknown") {
    throw std::runtime_error("initial_state must be 'empty' or 'unknown'");
  }
  config.simulate_attachment = parameter<bool>(node, "simulate_ideal_attachment", true);
  config.perception_source = parsePerception3dSource(
    parameter<std::string>(node, "perception_3d_source", "none"));
  config.depth_filtered_cloud_topic = parameter<std::string>(
    node, "depth_filtered_cloud_topic", "/x2/moveit/depth_filtered_cloud");
  config.lidar_filtered_cloud_topic = parameter<std::string>(
    node, "lidar_filtered_cloud_topic", "/x2/moveit/lidar_filtered_cloud");
  config.octomap_clear_service = parameter<std::string>(
    node, "octomap_clear_service", "/clear_octomap");
  config.octomap_refill_timeout = parameter<double>(node, "octomap_refill_timeout", 3.0);
  config.octomap_post_clear_samples = parameter<int>(node, "octomap_post_clear_samples", 2);
  config.maximum_filtered_cloud_age = parameter<double>(node, "maximum_filtered_cloud_age", 1.0);
  config.maximum_filtered_cloud_future_skew = parameter<double>(
    node, "maximum_filtered_cloud_future_skew", 0.10);
  config.octomap_processing_delay = parameter<double>(node, "octomap_processing_delay", 0.10);
  if (config.octomap_refill_timeout <= 0.0 || config.octomap_post_clear_samples < 1 ||
    config.maximum_filtered_cloud_age <= 0.0 || config.maximum_filtered_cloud_future_skew < 0.0 ||
    config.octomap_processing_delay < 0.0)
  {
    throw std::runtime_error(
            "OctoMap timeout/age must be positive, sample count at least one, and future "
            "skew/delay nonnegative");
  }
  config.attach_service = parameter<std::string>(node, "attach_service", "/mujoco_grasp/attach");
  config.detach_service = parameter<std::string>(node, "detach_service", "/mujoco_grasp/detach");

  if (config.ik_attempts_per_candidate < 1 || config.maximum_grasp_candidates < 1 ||
    config.maximum_planning_candidates < 1 || config.maximum_retry_candidates < 0 ||
    config.grasp_search_timeout <= 0.0 || config.planning_time_per_candidate <= 0.0 ||
    config.pregrasp_planning_timeout <= 0.0 || config.minimum_grasp_joint_margin < 0.0 ||
    config.closed_chain_position_tolerance < 0.0 ||
    config.closed_chain_orientation_tolerance < 0.0 || config.closed_chain_position_step <= 0.0 ||
    config.closed_chain_orientation_step <= 0.0 || config.closed_chain_ik_attempts < 1 ||
    config.closed_chain_ik_timeout <= 0.0 || config.closed_chain_beam_width < 1 ||
    config.closed_chain_solutions_per_branch < 1 || config.closed_chain_projection_limit < 1 ||
    config.closed_chain_validation_position_step <= 0.0 ||
    config.closed_chain_validation_orientation_step <= 0.0 ||
    config.closed_chain_contact_position_error < 0.0 ||
    config.closed_chain_contact_orientation_error < 0.0 || config.carry_search_timeout <= 0.0 ||
    config.maximum_carry_candidates < 1 || config.carry_search_x_range < 0.0 ||
    config.carry_search_y_range < 0.0 || config.carry_search_z_lower < 0.0 ||
    config.carry_search_z_upper < 0.0 || config.carry_search_orientation_tolerance < 0.0 ||
    (config.adaptive_place_position_tolerance.array() < 0.0).any() ||
    config.adaptive_place_yaw_tolerance < 0.0 || config.grasp_position_tolerance < 0.0 ||
    config.grasp_orientation_tolerance < 0.0 || config.pregrasp_distance_tolerance < 0.0 ||
    config.alternate_face_alignment_tolerance < 0.0)
  {
    throw std::runtime_error(
            "grasp search counts/timeouts must be positive and tolerances nonnegative");
  }
  return config;
}

}  // namespace agibot_x2_manipulation
