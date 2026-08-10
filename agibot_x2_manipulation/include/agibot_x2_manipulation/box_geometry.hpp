#pragma once

#include <Eigen/Geometry>

#include <array>

namespace agibot_x2_manipulation
{

struct BoxDimensions
{
  double length{0.30};
  double width{0.20};
  double height{0.15};
};

struct GraspGeometry
{
  Eigen::Isometry3d left_contact{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_contact{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d left_pregrasp{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_pregrasp{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d left_outward_normal{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d right_outward_normal{-Eigen::Vector3d::UnitY()};
  char selected_axis{'y'};
};

/// Convert a centered, aligned top-tag pose into the box-center pose.
Eigen::Isometry3d boxPoseFromTopTag(
  const Eigen::Isometry3d & tag_pose, const BoxDimensions & dimensions,
  double tag_to_box_yaw = 0.0);

/// Choose the face pair closest to base +/-Y and construct opposing TCP poses.
GraspGeometry computeGraspGeometry(
  const Eigen::Isometry3d & box_pose, const BoxDimensions & dimensions,
  double pregrasp_distance, double contact_height_offset = 0.0);

/// Interpolate a rigid object pose using linear translation and quaternion slerp.
Eigen::Isometry3d interpolatePose(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double t);

}  // namespace agibot_x2_manipulation
