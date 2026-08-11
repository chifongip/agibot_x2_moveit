#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <vector>

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

struct GraspCandidateOptions
{
  double position_tolerance{0.015};
  double orientation_tolerance{0.0872664626};
  double pregrasp_distance_tolerance{0.015};
  double alternate_face_alignment_tolerance{0.261799388};
  std::size_t maximum_candidates{64};
};

struct GraspCandidate
{
  GraspGeometry grasp;
  Eigen::Isometry3d planning_box_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d box_to_left_contact{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d box_to_right_contact{Eigen::Isometry3d::Identity()};
  double correction_cost{0.0};
  double tilt_correction{0.0};
  double contact_height_offset{0.0};
  double tangent_offset{0.0};
  double wrist_rotation{0.0};
  double pregrasp_distance{0.0};
};

/// Convert a centered, aligned top-tag pose into the box-center pose.
Eigen::Isometry3d boxPoseFromTopTag(
  const Eigen::Isometry3d & tag_pose, const BoxDimensions & dimensions,
  double tag_to_box_yaw = 0.0);

/// Choose the face pair closest to base +/-Y and construct opposing TCP poses.
GraspGeometry computeGraspGeometry(
  const Eigen::Isometry3d & box_pose, const BoxDimensions & dimensions,
  double pregrasp_distance, double contact_height_offset = 0.0);

/// Generate coordinated dual-arm grasp hypotheses around a measured box pose.
/// The measured top-center point and yaw are retained while bounded roll/pitch,
/// contact, wrist, clearance, and (near a diagonal) face-pair alternatives are searched.
std::vector<GraspCandidate> generateGraspCandidates(
  const Eigen::Isometry3d & measured_box_pose, const BoxDimensions & dimensions,
  double nominal_pregrasp_distance, double nominal_contact_height_offset,
  const GraspCandidateOptions & options);

/// Interpolate a rigid object pose using linear translation and quaternion slerp.
Eigen::Isometry3d interpolatePose(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double t);

}  // namespace agibot_x2_manipulation
