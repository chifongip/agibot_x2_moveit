#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>

#include <cmath>

namespace agibot_x2_manipulation
{

// Encoder feedback may be slightly outside model limits. Bound only the
// planning copy, using the same allowance as held-state admission. Execution
// still compares the trajectory start with the original measured feedback.
inline bool normalizePlanningStart(
  moveit::core::RobotState & state, const moveit::core::JointModelGroup * group,
  double tolerance)
{
  if (!group || !std::isfinite(tolerance) || tolerance < 0.0) {return false;}
  for (const auto & name : group->getVariableNames()) {
    if (!std::isfinite(state.getVariablePosition(name))) {return false;}
  }
  if (!state.satisfiesBounds(group, tolerance)) {return false;}
  state.enforceBounds(group);
  state.update();
  return true;
}

inline bool jointEndpointReached(
  const moveit::core::RobotState & actual, const moveit::core::RobotState & target,
  const moveit::core::JointModelGroup * group, double tolerance)
{
  if (!group || !std::isfinite(tolerance) || tolerance < 0.0) {return false;}
  for (const auto * joint : group->getActiveJointModels()) {
    for (std::size_t i = 0; i < joint->getVariableCount(); ++i) {
      if (!std::isfinite(actual.getJointPositions(joint)[i]) ||
        !std::isfinite(target.getJointPositions(joint)[i])) {return false;}
    }
    const double distance = joint->distance(
      actual.getJointPositions(joint), target.getJointPositions(joint));
    if (!std::isfinite(distance) || distance > tolerance) {return false;}
  }
  return true;
}

// Default to tight pose identity for skipping a planned segment. Physical
// contact checks pass the configured hardware accuracy tolerances explicitly.
inline bool endpointReached(
  const Eigen::Isometry3d & actual_left, const Eigen::Isometry3d & actual_right,
  const Eigen::Isometry3d & target_left, const Eigen::Isometry3d & target_right,
  double position_tolerance = 1e-4, double orientation_tolerance = 1e-3)
{
  if (!std::isfinite(position_tolerance) || position_tolerance < 0.0 ||
    !std::isfinite(orientation_tolerance) || orientation_tolerance < 0.0) {return false;}
  const auto reached = [=](const Eigen::Isometry3d & actual, const Eigen::Isometry3d & target) {
      return actual.matrix().allFinite() && target.matrix().allFinite() &&
             (actual.translation() - target.translation()).norm() <= position_tolerance &&
             std::abs(Eigen::AngleAxisd(actual.linear().transpose() * target.linear()).angle()) <=
             orientation_tolerance;
    };
  return reached(actual_left, target_left) && reached(actual_right, target_right);
}

}  // namespace agibot_x2_manipulation
