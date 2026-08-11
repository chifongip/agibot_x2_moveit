#include "agibot_x2_manipulation/box_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d contactPose(
  const Eigen::Isometry3d & box_pose, const Eigen::Vector3d & point_in_box,
  const Eigen::Vector3d & outward_in_box, bool left_hand, double wrist_rotation = 0.0)
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
  rotation = Eigen::AngleAxisd(
    left_hand ? wrist_rotation : -wrist_rotation, inward).toRotationMatrix() * rotation;

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = box_pose * point_in_box;
  result.linear() = rotation;
  return result;
}

GraspGeometry computeGraspGeometryForAxis(
  const Eigen::Isometry3d & box_pose, const BoxDimensions & dimensions,
  double pregrasp_distance, double contact_height_offset, double tangent_offset,
  double wrist_rotation, bool use_x)
{
  const Eigen::Vector3d base_y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d axis = use_x ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d tangent = use_x ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
  const double half_span = use_x ? dimensions.length / 2.0 : dimensions.width / 2.0;

  Eigen::Vector3d positive = axis;
  Eigen::Vector3d negative = -axis;
  if ((box_pose.linear() * positive).dot(base_y) < 0.0) {
    std::swap(positive, negative);
  }

  const Eigen::Vector3d offset =
    tangent * tangent_offset + Eigen::Vector3d::UnitZ() * contact_height_offset;
  GraspGeometry grasp;
  grasp.selected_axis = use_x ? 'x' : 'y';
  grasp.left_outward_normal = (box_pose.linear() * positive).normalized();
  grasp.right_outward_normal = (box_pose.linear() * negative).normalized();
  grasp.left_contact = contactPose(
    box_pose, positive * half_span + offset, positive, true, wrist_rotation);
  grasp.right_contact = contactPose(
    box_pose, negative * half_span + offset, negative, false, wrist_rotation);
  grasp.left_pregrasp = grasp.left_contact;
  grasp.right_pregrasp = grasp.right_contact;
  grasp.left_pregrasp.translation() += grasp.left_outward_normal * pregrasp_distance;
  grasp.right_pregrasp.translation() += grasp.right_outward_normal * pregrasp_distance;
  return grasp;
}

double tiltAngle(const Eigen::Matrix3d & rotation)
{
  return std::acos(std::clamp(
    (rotation * Eigen::Vector3d::UnitZ()).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
}

Eigen::Matrix3d uprightRotationWithMeasuredYaw(const Eigen::Matrix3d & rotation)
{
  Eigen::Vector3d x = rotation * Eigen::Vector3d::UnitX();
  x.z() = 0.0;
  if (x.head<2>().norm() < 1e-9) {
    Eigen::Vector3d y = rotation * Eigen::Vector3d::UnitY();
    y.z() = 0.0;
    x = Eigen::Vector3d(y.y(), -y.x(), 0.0);
  }
  x.normalize();
  Eigen::Matrix3d upright;
  upright.col(0) = x;
  upright.col(1) = Eigen::Vector3d::UnitZ().cross(x).normalized();
  upright.col(2) = Eigen::Vector3d::UnitZ();
  return upright;
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
  return computeGraspGeometryForAxis(
    box_pose, dimensions, pregrasp_distance, contact_height_offset, 0.0, 0.0, use_x);
}

std::vector<GraspCandidate> generateGraspCandidates(
  const Eigen::Isometry3d & measured_box_pose, const BoxDimensions & dimensions,
  double nominal_pregrasp_distance, double nominal_contact_height_offset,
  const GraspCandidateOptions & options)
{
  validate(dimensions);
  if (nominal_pregrasp_distance < 0.0 || options.position_tolerance < 0.0 ||
    options.orientation_tolerance < 0.0 || options.pregrasp_distance_tolerance < 0.0 ||
    options.alternate_face_alignment_tolerance < 0.0 || options.maximum_candidates == 0U)
  {
    throw std::invalid_argument("grasp candidate tolerances must be nonnegative");
  }

  const Eigen::Matrix3d measured_rotation = measured_box_pose.linear();
  const Eigen::Matrix3d upright_rotation = uprightRotationWithMeasuredYaw(measured_rotation);
  const double measured_tilt = tiltAngle(measured_rotation);
  const double maximum_correction = std::min(measured_tilt, options.orientation_tolerance);
  const Eigen::Quaterniond measured_q(measured_rotation);
  const Eigen::Quaterniond upright_q(upright_rotation);
  const Eigen::Vector3d measured_top_center = measured_box_pose.translation() +
    measured_rotation * Eigen::Vector3d(0.0, 0.0, dimensions.height / 2.0);

  const std::array<double, 3> factors{{0.0, 0.5, 1.0}};
  const std::array<double, 3> signed_factors{{0.0, -1.0, 1.0}};
  const double x_score = std::abs(
    (measured_rotation * Eigen::Vector3d::UnitX()).dot(Eigen::Vector3d::UnitY()));
  const double y_score = std::abs(
    (measured_rotation * Eigen::Vector3d::UnitY()).dot(Eigen::Vector3d::UnitY()));
  const bool nominal_use_x = x_score > y_score + 1e-12;
  const double nominal_angle = std::acos(std::clamp(std::max(x_score, y_score), 0.0, 1.0));
  const double alternate_angle = std::acos(std::clamp(std::min(x_score, y_score), 0.0, 1.0));
  std::vector<bool> axes{nominal_use_x};
  if (alternate_angle - nominal_angle <= options.alternate_face_alignment_tolerance + 1e-12) {
    axes.push_back(!nominal_use_x);
  }

  std::vector<GraspCandidate> candidates;
  for (const bool use_x : axes) {
    for (const double tilt_factor : factors) {
      const double tilt_correction = tilt_factor * maximum_correction;
      const double interpolation = measured_tilt > 1e-12 ? tilt_correction / measured_tilt : 0.0;
      Eigen::Isometry3d planning_box_pose = Eigen::Isometry3d::Identity();
      planning_box_pose.linear() = measured_q.slerp(interpolation, upright_q).normalized().toRotationMatrix();
      planning_box_pose.translation() = measured_top_center -
        planning_box_pose.linear() * Eigen::Vector3d(0.0, 0.0, dimensions.height / 2.0);
      for (const double height_factor : signed_factors) {
        const double contact_height =
          nominal_contact_height_offset + height_factor * options.position_tolerance;
        if (std::abs(contact_height) >= dimensions.height / 2.0) {
          continue;
        }
        for (const double tangent_factor : signed_factors) {
          const double tangent_offset = tangent_factor * options.position_tolerance;
          const double tangent_half_span = use_x ? dimensions.width / 2.0 : dimensions.length / 2.0;
          if (std::abs(tangent_offset) >= tangent_half_span) {
            continue;
          }
          for (const double wrist_factor : signed_factors) {
            const double wrist_rotation = wrist_factor * options.orientation_tolerance;
            for (const double clearance_factor : signed_factors) {
              const double pregrasp_distance = nominal_pregrasp_distance +
                clearance_factor * options.pregrasp_distance_tolerance;
              if (pregrasp_distance < 0.0) {
                continue;
              }
              GraspCandidate candidate;
              candidate.planning_box_pose = planning_box_pose;
              candidate.grasp = computeGraspGeometryForAxis(
                planning_box_pose, dimensions, pregrasp_distance, contact_height,
                tangent_offset, wrist_rotation, use_x);
              candidate.box_to_left_contact =
                measured_box_pose.inverse() * candidate.grasp.left_contact;
              candidate.box_to_right_contact =
                measured_box_pose.inverse() * candidate.grasp.right_contact;
              candidate.tilt_correction = tilt_correction;
              candidate.contact_height_offset = contact_height;
              candidate.tangent_offset = tangent_offset;
              candidate.wrist_rotation = wrist_rotation;
              candidate.pregrasp_distance = pregrasp_distance;
              const double face_penalty = use_x == nominal_use_x ? 0.0 : 1.0;
              candidate.correction_cost = face_penalty + tilt_factor +
                std::abs(height_factor) + std::abs(tangent_factor) +
                std::abs(wrist_factor) + std::abs(clearance_factor);
              candidates.push_back(std::move(candidate));
            }
          }
        }
      }
    }
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const GraspCandidate & lhs, const GraspCandidate & rhs) {
      return std::tie(lhs.correction_cost, lhs.tilt_correction) <
             std::tie(rhs.correction_cost, rhs.tilt_correction);
    });
  if (candidates.size() > options.maximum_candidates) {
    candidates.resize(options.maximum_candidates);
  }
  return candidates;
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
