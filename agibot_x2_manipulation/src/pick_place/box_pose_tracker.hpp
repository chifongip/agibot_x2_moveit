#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <mutex>
#include <string>

namespace agibot_x2_manipulation
{

class BoxPoseTracker
{
public:
  BoxPoseTracker(
    const rclcpp::Node::SharedPtr & node, std::string planning_frame,
    std::string topic, double maximum_age, double position_tolerance,
    double orientation_tolerance);

  bool stablePose(geometry_msgs::msg::PoseStamped & pose) const;
  bool stillWithinTolerance(
    const Eigen::Isometry3d & reference, geometry_msgs::msg::PoseStamped & latest,
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
  bool have_pose_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

}  // namespace agibot_x2_manipulation
