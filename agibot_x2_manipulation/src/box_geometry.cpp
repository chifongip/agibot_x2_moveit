#include "agibot_x2_manipulation/box_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d contactPose(
  const Eigen::Isometry3d & box_pose, const Eigen::Vector3d & point_in_box,
  const Eigen::Vector3d & outward_in_box, bool left_hand)
{
  const Eigen::Vector3d outward = box_pose.linear() * outward_in_box;
  const Eigen::Vector3d inward = -outward.normalized();
  const Eigen::Vector3d up = (box_pose.linear() * Eigen::Vector3d::UnitZ()).normalized();
  // TCP +X points up. Right TCP +Y and left TCP -Y point inward through the
  // contact surface.
  const Eigen::Vector3d y_axis = left_hand ? outward.normalized() : inward;
  const Eigen::Vector3d x_axis = up;
  const Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = box_pose * point_in_box;
  result.linear() = rotation;
  return result;
}

void validate(const BoxDimensions & dimensions)
{
  if (dimensions.length <= 0.0 || dimensions.width <= 0.0 || dimensions.height <= 0.0) {
    throw std::invalid_argument("box dimensions must be positive");
  }
}

}  // namespace

Eigen::Isometry3d boxPoseFromTopTag(
  const Eigen::Isometry3d & tag_pose, const BoxDimensions & dimensions,
  double tag_to_box_yaw)
{
  validate(dimensions);
  Eigen::Isometry3d tag_to_box = Eigen::Isometry3d::Identity();
  tag_to_box.linear() = Eigen::AngleAxisd(tag_to_box_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tag_to_box.translation() = Eigen::Vector3d(0.0, 0.0, -dimensions.height / 2.0);
  return tag_pose * tag_to_box;
}

GraspGeometry computeGraspGeometry(
  const Eigen::Isometry3d & box_pose, const BoxDimensions & dimensions,
  double pregrasp_distance, double contact_height_offset)
{
  validate(dimensions);
  if (pregrasp_distance < 0.0) {
    throw std::invalid_argument("pregrasp distance cannot be negative");
  }
  if (std::abs(contact_height_offset) >= dimensions.height / 2.0) {
    throw std::invalid_argument("contact height lies outside the box");
  }

  const Eigen::Vector3d base_y = Eigen::Vector3d::UnitY();
  const double x_score = std::abs((box_pose.linear() * Eigen::Vector3d::UnitX()).dot(base_y));
  const double y_score = std::abs((box_pose.linear() * Eigen::Vector3d::UnitY()).dot(base_y));
  const bool use_x = x_score > y_score + 1e-12;  // Exact ties deliberately select local Y.
  const Eigen::Vector3d axis = use_x ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  const double half_span = use_x ? dimensions.length / 2.0 : dimensions.width / 2.0;

  Eigen::Vector3d positive = axis;
  Eigen::Vector3d negative = -axis;
  if ((box_pose.linear() * positive).dot(base_y) < 0.0) {
    std::swap(positive, negative);
  }

  const Eigen::Vector3d center_z(0.0, 0.0, contact_height_offset);
  GraspGeometry grasp;
  grasp.selected_axis = use_x ? 'x' : 'y';
  grasp.left_outward_normal = (box_pose.linear() * positive).normalized();
  grasp.right_outward_normal = (box_pose.linear() * negative).normalized();
  grasp.left_contact = contactPose(box_pose, positive * half_span + center_z, positive, true);
  grasp.right_contact = contactPose(box_pose, negative * half_span + center_z, negative, false);
  grasp.left_pregrasp = grasp.left_contact;
  grasp.right_pregrasp = grasp.right_contact;
  grasp.left_pregrasp.translation() += grasp.left_outward_normal * pregrasp_distance;
  grasp.right_pregrasp.translation() += grasp.right_outward_normal * pregrasp_distance;
  return grasp;
}

Eigen::Isometry3d interpolatePose(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double t)
{
  t = std::clamp(t, 0.0, 1.0);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - t) * from.translation() + t * to.translation();
  const Eigen::Quaterniond q_from(from.linear());
  const Eigen::Quaterniond q_to(to.linear());
  result.linear() = q_from.slerp(t, q_to).normalized().toRotationMatrix();
  return result;
}

}  // namespace agibot_x2_manipulation
