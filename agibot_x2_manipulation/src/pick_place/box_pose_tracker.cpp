#include "pick_place/box_pose_tracker.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
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

TableTagPoseStabilityFilter::TableTagPoseStabilityFilter(
  std::size_t stable_sample_count, double maximum_position_spread,
  double maximum_angular_spread, double maximum_sample_gap)
: stable_sample_count_(stable_sample_count),
  maximum_position_spread_(maximum_position_spread),
  maximum_angular_spread_(maximum_angular_spread),
  maximum_sample_gap_(rclcpp::Duration::from_seconds(maximum_sample_gap))
{
}

TableTagPoseStabilityUpdate TableTagPoseStabilityFilter::addSample(
  const Eigen::Isometry3d & sample, const rclcpp::Time & stamp)
{
  if (!sample.matrix().allFinite()) {
    return {};
  }
  if (!samples_.empty()) {
    const auto & previous_stamp = samples_.back().stamp;
    if (stamp <= previous_stamp) {
      return {};
    }
    if ((stamp - previous_stamp) > maximum_sample_gap_) {
      samples_.clear();
    }
  }

  samples_.push_back({sample, stamp});
  while (samples_.size() > stable_sample_count_) {
    samples_.pop_front();
  }
  if (samples_.size() != stable_sample_count_) {
    return {true, std::nullopt};
  }

  Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference(samples_.front().pose.linear());
  for (const auto & item : samples_) {
    mean_position += item.pose.translation();
    Eigen::Quaterniond quaternion(item.pose.linear());
    if (quaternion.dot(reference) < 0.0) {
      quaternion.coeffs() *= -1.0;
    }
    quaternion_sum += quaternion.coeffs();
  }
  if (quaternion_sum.norm() < 1e-9) {
    return {true, std::nullopt};
  }
  mean_position /= static_cast<double>(samples_.size());
  Eigen::Quaterniond mean_quaternion;
  mean_quaternion.coeffs() = quaternion_sum.normalized();

  double position_spread = 0.0;
  double angular_spread = 0.0;
  for (const auto & item : samples_) {
    position_spread = std::max(
      position_spread, (item.pose.translation() - mean_position).norm());
    const Eigen::Quaterniond quaternion(item.pose.linear());
    angular_spread = std::max(
      angular_spread, 2.0 * std::acos(
        std::clamp(std::abs(quaternion.dot(mean_quaternion)), 0.0, 1.0)));
  }
  if (position_spread > maximum_position_spread_ ||
    angular_spread > maximum_angular_spread_)
  {
    return {true, std::nullopt};
  }

  Eigen::Isometry3d stable_pose = Eigen::Isometry3d::Identity();
  stable_pose.translation() = mean_position;
  stable_pose.linear() = mean_quaternion.toRotationMatrix();
  return {true, stable_pose};
}

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

TableTagPlacePoseTracker::TableTagPlacePoseTracker(
  const rclcpp::Node::SharedPtr & node, std::string planning_frame,
  std::string tag_frame, std::string detections_topic, int tag_id,
  double minimum_decision_margin, const BoxDimensions & dimensions,
  double tag_height_above_tabletop, double table_x_offset, double table_z_offset,
  double tag_to_box_yaw, std::size_t stable_sample_count, double maximum_age,
  double maximum_position_spread, double maximum_angular_spread,
  double maximum_sample_gap)
: node_(node), planning_frame_(std::move(planning_frame)), tag_frame_(std::move(tag_frame)),
  tag_id_(tag_id), minimum_decision_margin_(minimum_decision_margin), dimensions_(dimensions),
  tag_height_above_tabletop_(tag_height_above_tabletop), table_x_offset_(table_x_offset),
  table_z_offset_(table_z_offset), tag_to_box_yaw_(tag_to_box_yaw),
  maximum_age_(maximum_age), stability_filter_(
    stable_sample_count, maximum_position_spread, maximum_angular_spread, maximum_sample_gap),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{
  detections_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    std::move(detections_topic), rclcpp::SensorDataQoS(),
    std::bind(&TableTagPlacePoseTracker::onDetections, this, std::placeholders::_1));
}

void TableTagPlacePoseTracker::onDetections(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message)
{
  const auto detection = std::find_if(
    message->detections.begin(), message->detections.end(),
    [this](const auto & item) {
      return item.id == tag_id_ && item.decision_margin >= minimum_decision_margin_;
    });
  if (detection == message->detections.end()) {
    return;
  }

  try {
    const rclcpp::Time detection_stamp(message->header.stamp);
    if (detection_stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Table tag detection rejected because its timestamp is zero");
      return;
    }
    if ((node_->now() - detection_stamp).seconds() > maximum_age_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Table tag detection rejected because it is older than %.3f s", maximum_age_);
      return;
    }
    const auto transform = tf_buffer_.lookupTransform(
      planning_frame_, tag_frame_, detection_stamp);
    const Eigen::Isometry3d sample = boxPoseFromVerticalTableTag(
      tf2::transformToEigen(transform), dimensions_, tag_height_above_tabletop_,
      table_x_offset_, table_z_offset_, tag_to_box_yaw_);
    updateStablePose(sample, message->header.stamp);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Table tag TF unavailable: %s", error.what());
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Table tag pose rejected: %s", error.what());
  }
}

void TableTagPlacePoseTracker::updateStablePose(
  const Eigen::Isometry3d & sample, const builtin_interfaces::msg::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto update = stability_filter_.addSample(sample, rclcpp::Time(stamp));
  if (!update.accepted_sample) {
    return;
  }
  have_stable_pose_ = false;
  if (!update.stable_pose) {
    return;
  }

  const Eigen::Isometry3d & stable_sample = *update.stable_pose;
  stable_pose_.header.frame_id = planning_frame_;
  stable_pose_.header.stamp = stamp;
  stable_pose_.pose.position.x = stable_sample.translation().x();
  stable_pose_.pose.position.y = stable_sample.translation().y();
  stable_pose_.pose.position.z = stable_sample.translation().z();
  stable_pose_.pose.orientation = tf2::toMsg(Eigen::Quaterniond(stable_sample.linear()));
  have_stable_pose_ = true;
  stable_pose_condition_.notify_all();
}

bool TableTagPlacePoseTracker::waitForStablePose(
  double timeout, const std::function<bool()> & canceled,
  geometry_msgs::msg::PoseStamped & output, std::string & error) const
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (canceled()) {
      error = "waiting for stable table tag pose canceled";
      return false;
    }
    if (have_stable_pose_ &&
      (node_->now() - stable_pose_.header.stamp).seconds() <= maximum_age_)
    {
      output = stable_pose_;
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      error = "no fresh stable table tag pose";
      return false;
    }
    stable_pose_condition_.wait_for(
      lock, std::min(std::chrono::milliseconds(100),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
  }
}

void BoxPoseTracker::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_pose_ = false;
}

}  // namespace agibot_x2_manipulation
