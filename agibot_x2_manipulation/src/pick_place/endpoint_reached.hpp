#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>

#include <cmath>

namespace agibot_x2_manipulation
{

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

// Deliberately tighter than grasp accuracy tolerances: skipping a segment must
// not erase a requested small translation or rotation.
inline bool endpointReached(
  const Eigen::Isometry3d & actual_left, const Eigen::Isometry3d & actual_right,
  const Eigen::Isometry3d & target_left, const Eigen::Isometry3d & target_right)
{
  const auto reached = [](const Eigen::Isometry3d & actual, const Eigen::Isometry3d & target) {
      return actual.matrix().allFinite() && target.matrix().allFinite() &&
             (actual.translation() - target.translation()).norm() <= 1e-4 &&
             std::abs(Eigen::AngleAxisd(actual.linear().transpose() * target.linear()).angle()) <=
             1e-3;
    };
  return reached(actual_left, target_left) && reached(actual_right, target_right);
}

}  // namespace agibot_x2_manipulation
