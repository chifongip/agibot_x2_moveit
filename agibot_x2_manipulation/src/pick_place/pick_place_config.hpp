#pragma once

#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/perception_readiness.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace agibot_x2_manipulation
{

enum class MotionPlanningMode
{
  CLOSED_CHAIN,
  POSE_TO_POSE
};

MotionPlanningMode parseMotionPlanningMode(const std::string & value);
const char * motionPlanningModeName(MotionPlanningMode mode);

struct PickPlaceConfig
{
  std::string planning_frame;
  std::string box_pose_topic;
  std::string planning_group;
  std::string left_group_name;
  std::string right_group_name;
  std::string left_tcp;
  std::string right_tcp;
  std::string motion_planning_mode_name;
  MotionPlanningMode motion_planning_mode{MotionPlanningMode::CLOSED_CHAIN};
  std::string box_id;
  BoxDimensions dimensions;
  double pregrasp_distance{0.0};
  double contact_height_offset{0.0};
  double lift_height{0.0};
  double cartesian_step{0.0};
  double max_pose_age{0.0};
  double ik_timeout{0.0};
  double grasp_position_tolerance{0.0};
  double grasp_orientation_tolerance{0.0};
  double pregrasp_distance_tolerance{0.0};
  double alternate_face_alignment_tolerance{0.0};
  int maximum_grasp_candidates{0};
  double grasp_search_timeout{0.0};
  int ik_attempts_per_candidate{0};
  int maximum_planning_candidates{0};
  double planning_time_per_candidate{0.0};
  double pregrasp_planning_timeout{0.0};
  int maximum_retry_candidates{0};
  double minimum_grasp_joint_margin{0.0};
  double closed_chain_position_tolerance{0.0};
  double closed_chain_orientation_tolerance{0.0};
  double closed_chain_position_step{0.0};
  double closed_chain_orientation_step{0.0};
  int closed_chain_ik_attempts{0};
  double closed_chain_ik_timeout{0.0};
  int closed_chain_beam_width{0};
  int closed_chain_solutions_per_branch{0};
  int closed_chain_projection_limit{0};
  double closed_chain_validation_position_step{0.0};
  double closed_chain_validation_orientation_step{0.0};
  double closed_chain_contact_position_error{0.0};
  double closed_chain_contact_orientation_error{0.0};
  double carry_search_timeout{0.0};
  int maximum_carry_candidates{0};
  double carry_search_x_range{0.0};
  double carry_search_y_range{0.0};
  double carry_search_z_lower{0.0};
  double carry_search_z_upper{0.0};
  double carry_search_orientation_tolerance{0.0};
  Eigen::Vector3d adaptive_place_position_tolerance{Eigen::Vector3d::Zero()};
  double adaptive_place_yaw_tolerance{0.0};
  double max_joint_step{0.0};
  bool allow_execution{false};
  double velocity_scaling{0.0};
  double acceleration_scaling{0.0};
  Eigen::Isometry3d carry_pose{Eigen::Isometry3d::Identity()};
  double recovery_position_tolerance{0.0};
  double recovery_angular_tolerance{0.0};
  std::string state_file;
  std::string initial_state;
  std::string post_place_named_target;
  std::string reset_named_target;
  double reset_preemption_timeout{0.0};
  double reset_state_timeout{0.0};
  double reset_joint_tolerance{0.0};
  double execution_settle_timeout{0.0};
  double execution_joint_tolerance{0.0};
  double execution_velocity_tolerance{0.0};
  int execution_settle_samples{0};
  std::string arm_state_topic;
  bool simulate_attachment{false};
  std::string attach_service;
  std::string detach_service;
  Perception3dSource perception_source{Perception3dSource::NONE};
  std::string depth_filtered_cloud_topic;
  std::string lidar_filtered_cloud_topic;
  std::string octomap_clear_service;
  double octomap_refill_timeout{0.0};
  int octomap_post_clear_samples{0};
  double maximum_filtered_cloud_age{0.0};
  double maximum_filtered_cloud_future_skew{0.0};
  double octomap_processing_delay{0.0};
};

PickPlaceConfig loadPickPlaceConfig(const rclcpp::Node::SharedPtr & node);

}  // namespace agibot_x2_manipulation
