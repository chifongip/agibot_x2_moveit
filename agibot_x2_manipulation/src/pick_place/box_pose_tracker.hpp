#pragma once

#include "agibot_x2_manipulation/box_geometry.hpp"

#include <agibot_x2_manipulation_msgs/msg/box_state_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace agibot_x2_manipulation
{

struct TrackedBoxPose
{
  std::string instance_id;
  std::string profile_id;
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
};

class BoxPoseTracker
{
public:
  BoxPoseTracker(
    const rclcpp::Node::SharedPtr & node, std::string planning_frame,
    std::string legacy_topic, std::string states_topic, double maximum_age,
    double position_tolerance,
    double orientation_tolerance);

  bool stablePose(const std::string & instance_id, TrackedBoxPose & pose) const;
  std::map<std::string, TrackedBoxPose> freshPoses() const;
  bool stillWithinTolerance(
    const TrackedBoxPose & reference, TrackedBoxPose & latest,
    std::string & error) const;
  bool transformGoalPose(
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output,
    std::string & error) const;
  void clear();

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  double maximum_age_;
  double position_tolerance_;
  double orientation_tolerance_;
  mutable std::mutex mutex_;
  std::map<std::string, TrackedBoxPose> latest_poses_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr legacy_subscription_;
  rclcpp::Subscription<agibot_x2_manipulation_msgs::msg::BoxStateArray>::SharedPtr
  states_subscription_;
};

struct TableTagPoseStabilityUpdate
{
  bool accepted_sample{false};
  std::optional<Eigen::Isometry3d> stable_pose;
};

class TableTagPoseStabilityFilter
{
public:
  TableTagPoseStabilityFilter(
    std::size_t stable_sample_count, double maximum_position_spread,
    double maximum_angular_spread, double maximum_sample_gap);

  TableTagPoseStabilityUpdate addSample(
    const Eigen::Isometry3d & sample, const rclcpp::Time & stamp);

private:
  struct Sample
  {
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  std::size_t stable_sample_count_;
  double maximum_position_spread_;
  double maximum_angular_spread_;
  rclcpp::Duration maximum_sample_gap_;
  std::deque<Sample> samples_;
};

class TableTagPoseTracker
{
public:
  using StablePoseCallback = std::function<void(const geometry_msgs::msg::PoseStamped &)>;

  TableTagPoseTracker(
    const rclcpp::Node::SharedPtr & node, std::string planning_frame,
    std::string tag_frame, std::string detections_topic, int tag_id,
    double minimum_decision_margin, std::size_t stable_sample_count, double maximum_age,
    double maximum_position_spread, double maximum_angular_spread,
    double maximum_sample_gap, StablePoseCallback stable_pose_callback = {});

  bool waitForStablePose(
    double timeout, const std::function<bool()> & canceled,
    geometry_msgs::msg::PoseStamped & output, std::string & error) const;

private:
  void onDetections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message);
  void updateStablePose(
    const Eigen::Isometry3d & sample, const builtin_interfaces::msg::Time & stamp);

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string tag_frame_;
  int tag_id_;
  double minimum_decision_margin_;
  double maximum_age_;
  mutable std::mutex mutex_;
  mutable std::condition_variable stable_pose_condition_;
  TableTagPoseStabilityFilter stability_filter_;
  bool have_stable_pose_{false};
  geometry_msgs::msg::PoseStamped stable_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  StablePoseCallback stable_pose_callback_;
};

}  // namespace agibot_x2_manipulation
