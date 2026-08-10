#include "agibot_x2_manipulation/box_geometry.hpp"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{

class BoxLocalizer : public rclcpp::Node
{
public:
  BoxLocalizer()
  : Node("box_localizer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter("planning_frame", "base_link");
    tag_frame_ = declare_parameter("tag_frame", "tag0");
    const auto topic = declare_parameter("detections_topic", "/detections");
    tag_id_ = declare_parameter("tag_id", 0);
    const auto dimensions = declare_parameter<std::vector<double>>(
      "box_dimensions", {0.30, 0.20, 0.15});
    if (dimensions.size() != 3U) {
      throw std::runtime_error("box_dimensions must contain [length, width, height]");
    }
    dimensions_ = {dimensions[0], dimensions[1], dimensions[2]};
    tag_to_box_yaw_ = declare_parameter("tag_to_box_yaw", 0.0);
    stable_count_ = static_cast<std::size_t>(declare_parameter("stable_sample_count", 10));
    max_age_ = declare_parameter("maximum_pose_age", 0.25);
    max_position_spread_ = declare_parameter("maximum_position_spread", 0.005);
    max_angular_spread_ = declare_parameter("maximum_angular_spread", 0.0523598776);
    max_box_tilt_ = declare_parameter("maximum_box_tilt", 0.0872664626);
    minimum_margin_ = declare_parameter("minimum_decision_margin", 20.0);
    if (stable_count_ < 2U) {
      throw std::runtime_error("stable_sample_count must be at least 2");
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/box_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/box_markers", 10);
    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&BoxLocalizer::onDetections, this, std::placeholders::_1));
  }

private:
  void onDetections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message)
  {
    const auto detection = std::find_if(
      message->detections.begin(), message->detections.end(),
      [this](const auto & item) {
        return item.id == tag_id_ && item.decision_margin >= minimum_margin_;
      });
    if (detection == message->detections.end()) {
      return;
    }

    try {
      const auto tf = tf_buffer_.lookupTransform(planning_frame_, tag_frame_, tf2::TimePointZero);
      const rclcpp::Time tf_stamp(tf.header.stamp);
      if ((now() - tf_stamp).seconds() > max_age_) {
        return;
      }
      const Eigen::Isometry3d tag_pose = tf2::transformToEigen(tf);
      const Eigen::Isometry3d box_pose = boxPoseFromTopTag(tag_pose, dimensions_, tag_to_box_yaw_);
      const Eigen::Vector3d box_up = box_pose.linear() * Eigen::Vector3d::UnitZ();
      const double tilt = std::acos(std::clamp(box_up.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
      if (tilt > max_box_tilt_) {
        samples_.clear();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Box tilt %.1f deg exceeds upright limit",
          tilt * 180.0 / 3.14159265358979323846);
        return;
      }
      samples_.push_back(box_pose);
      while (samples_.size() > stable_count_) {
        samples_.pop_front();
      }
      if (samples_.size() == stable_count_) {
        publishIfStable(tf.header.stamp);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Tag TF unavailable: %s", error.what());
    }
  }

  void publishIfStable(const builtin_interfaces::msg::Time & stamp)
  {
    Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
    Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
    const Eigen::Quaterniond reference(samples_.front().linear());
    for (const auto & sample : samples_) {
      mean_position += sample.translation();
      Eigen::Quaterniond quaternion(sample.linear());
      if (quaternion.dot(reference) < 0.0) {
        quaternion.coeffs() *= -1.0;
      }
      quaternion_sum += quaternion.coeffs();
    }
    mean_position /= static_cast<double>(samples_.size());
    Eigen::Quaterniond mean_quaternion;
    mean_quaternion.coeffs() = quaternion_sum.normalized();

    double position_spread = 0.0;
    double angular_spread = 0.0;
    for (const auto & sample : samples_) {
      position_spread = std::max(position_spread, (sample.translation() - mean_position).norm());
      const Eigen::Quaterniond q(sample.linear());
      angular_spread = std::max(
        angular_spread, 2.0 * std::acos(std::clamp(std::abs(q.dot(mean_quaternion)), 0.0, 1.0)));
    }
    if (position_spread > max_position_spread_ || angular_spread > max_angular_spread_) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped output;
    output.header.frame_id = planning_frame_;
    output.header.stamp = stamp;
    output.pose.pose.position.x = mean_position.x();
    output.pose.pose.position.y = mean_position.y();
    output.pose.pose.position.z = mean_position.z();
    output.pose.pose.orientation = tf2::toMsg(mean_quaternion);
    const double position_variance = position_spread * position_spread;
    const double angular_variance = angular_spread * angular_spread;
    output.pose.covariance[0] = output.pose.covariance[7] = output.pose.covariance[14] = position_variance;
    output.pose.covariance[21] = output.pose.covariance[28] = output.pose.covariance[35] = angular_variance;
    pose_pub_->publish(output);

    visualization_msgs::msg::Marker marker;
    marker.header = output.header;
    marker.ns = "localized_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = output.pose.pose;
    marker.scale.x = dimensions_.length;
    marker.scale.y = dimensions_.width;
    marker.scale.z = dimensions_.height;
    marker.color.r = 0.2F;
    marker.color.g = 0.7F;
    marker.color.b = 0.9F;
    marker.color.a = 0.35F;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  std::string planning_frame_;
  std::string tag_frame_;
  int tag_id_;
  BoxDimensions dimensions_;
  double tag_to_box_yaw_;
  std::size_t stable_count_;
  double max_age_;
  double max_position_spread_;
  double max_angular_spread_;
  double max_box_tilt_;
  double minimum_margin_;
  std::deque<Eigen::Isometry3d> samples_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agibot_x2_manipulation::BoxLocalizer>());
  rclcpp::shutdown();
  return 0;
}
