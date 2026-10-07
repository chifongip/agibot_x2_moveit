#pragma once

#include "pick_place/post_place_planner.hpp"

#include <optional>

namespace agibot_x2_manipulation
{

Eigen::Isometry3d pickLiftTarget(
  const Eigen::Isometry3d & measured, double lift_height,
  std::optional<double> original_top = std::nullopt);

// Shared progress for two tip paths; unlike rigid multi-tip motion this also
// supports the changing hand separation during approach and withdrawal.
bool validateCartesianState(
  const HandPosePair & actual, const HandPosePair & from, const HandPosePair & to,
  double position_tolerance, double orientation_tolerance, std::string & error);

bool validateCartesianTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, const PickPlaceConfig & config,
  const HandPosePair & from, const HandPosePair & to, std::string & error,
  const CancelFunction & interrupted, double minimum_joint_margin = 0.0);

// Quintic segments with zero endpoint velocity/acceleration stay between joint
// waypoint positions. Used only when normal timing overshoots position bounds.
bool retimeCartesianWithoutOvershoot(
  robot_trajectory::RobotTrajectory & path, const PickPlaceConfig & config, std::string & error,
  const CancelFunction & interrupted = {});

struct CartesianRepairInfo
{
  std::string strategy;
  std::string fallback_reason;
  std::vector<std::string> joints;
  std::vector<std::size_t> intervals;
  double original_duration{0.0};
  double final_duration{0.0};
};

// Transactional repair: preserve positions and try original timing first.
bool repairCartesianTrajectory(
  robot_trajectory::RobotTrajectory & path,
  const planning_scene::PlanningSceneConstPtr & scene, const PickPlaceConfig & config,
  const HandPosePair & from, const HandPosePair & to, std::string & error,
  const CancelFunction & interrupted, double minimum_joint_margin = 0.0,
  CartesianRepairInfo * info = nullptr);

bool planCartesianMotion(
  const moveit::core::RobotState & start, const HandPosePair & target,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end,
  std::string & error, const CancelFunction & canceled, PlanningDeadline deadline,
  double minimum_joint_margin = 0.0);

bool isCartesianPickSegment(const std::string & segment);

}  // namespace agibot_x2_manipulation
