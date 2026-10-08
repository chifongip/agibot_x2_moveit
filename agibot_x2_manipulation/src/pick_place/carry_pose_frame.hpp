#pragma once

#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <stdexcept>
#include <string>

namespace agibot_x2_manipulation
{

// Resolve from the planning snapshot, so torso feedback and arm IK agree.
inline Eigen::Isometry3d carryFrameTransform(
  const moveit::core::RobotState & state, const std::string & planning_frame)
{
  if (state.dirtyLinkTransforms()) {
    // MoveGroup feedback may contain fresh joints with lazy FK still pending.
    // Update a copy without mutating the caller's planning snapshot.
    moveit::core::RobotState updated(state);
    updated.update();
    return carryFrameTransform(updated, planning_frame);
  }
  if (!state.getRobotModel()->hasLinkModel("torso_link")) {
    throw std::runtime_error("carry reference link torso_link is absent from robot model");
  }
  Eigen::Isometry3d model_to_planning = Eigen::Isometry3d::Identity();
  if (planning_frame != state.getRobotModel()->getModelFrame()) {
    if (!state.getRobotModel()->hasLinkModel(planning_frame)) {
      throw std::runtime_error("carry planning frame is absent from robot model: " + planning_frame);
    }
    model_to_planning = state.getGlobalLinkTransform(planning_frame);
  }
  return model_to_planning.inverse() * state.getGlobalLinkTransform("torso_link");
}

inline Eigen::Isometry3d carryPoseInPlanningFrame(
  const moveit::core::RobotState & state, const std::string & planning_frame,
  const Eigen::Isometry3d & torso_pose)
{
  return carryFrameTransform(state, planning_frame) * torso_pose;
}

inline bool carryFrameMatches(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & reference,
  double position_tolerance, double angular_tolerance)
{
  return (actual.translation() - reference.translation()).norm() <= position_tolerance &&
         Eigen::Quaterniond(actual.linear()).angularDistance(
    Eigen::Quaterniond(reference.linear())) <= angular_tolerance;
}

}  // namespace agibot_x2_manipulation
