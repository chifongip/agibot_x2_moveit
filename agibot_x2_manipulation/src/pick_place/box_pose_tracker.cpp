#include "pick_place/box_pose_tracker.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (rotation.norm() < 1e-9) {
    throw std::invalid_argument("pose quaternion has zero length");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

}  // namespace

BoxPoseTracker::BoxPoseTracker(
  const rclcpp::Node::SharedPtr & node, std::string planning_frame,
  std::string topic, double maximum_age, double position_tolerance,
  double orientation_tolerance)
: node_(node), planning_frame_(std::move(planning_frame)), maximum_age_(maximum_age),
  position_tolerance_(position_tolerance), orientation_tolerance_(orientation_tolerance),
  tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
{
  subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    std::move(topic), 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_pose_ = *message;
      have_pose_ = true;
    });
}

bool BoxPoseTracker::stablePose(geometry_msgs::msg::PoseStamped & pose) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_pose_ || (node_->now() - latest_pose_.header.stamp).seconds() > maximum_age_) {
    return false;
  }
  pose.header = latest_pose_.header;
  pose.pose = latest_pose_.pose.pose;
  return pose.header.frame_id == planning_frame_;
}

bool BoxPoseTracker::stillWithinTolerance(
  const Eigen::Isometry3d & reference, geometry_msgs::msg::PoseStamped & latest,
  std::string & error) const
{
  if (!stablePose(latest)) {
    error = "box pose became stale before approach";
    return false;
  }
  Eigen::Isometry3d current;
  try {
    current = toEigen(latest.pose);
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
  const double position_error = (current.translation() - reference.translation()).norm();
  const Eigen::Quaterniond reference_q(reference.linear());
  const Eigen::Quaterniond current_q(current.linear());
  const double angular_error = 2.0 * std::acos(
    std::clamp(std::abs(reference_q.dot(current_q)), 0.0, 1.0));
  if (position_error > position_tolerance_ || angular_error > orientation_tolerance_) {
    error = "box moved after planning (position=" + std::to_string(position_error) +
      " m, angle=" + std::to_string(angular_error) + " rad)";
    return false;
  }
  return true;
}

bool BoxPoseTracker::transformGoalPose(
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output,
  std::string & error) const
{
  if (input.header.frame_id.empty()) {
    error = "place_pose.frame_id is empty";
    return false;
  }
  try {
    if (input.header.frame_id == planning_frame_) {
      output = input;
    } else {
      const auto transform = tf_buffer_.lookupTransform(
        planning_frame_, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
    }
    output.header.frame_id = planning_frame_;
    return true;
  } catch (const tf2::TransformException & exception) {
    error = exception.what();
    return false;
  }
}

void BoxPoseTracker::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_pose_ = false;
}

}  // namespace agibot_x2_manipulation
