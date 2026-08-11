#include "agibot_x2_manipulation/box_geometry.hpp"

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{

class DummyAprilTag : public rclcpp::Node
{
public:
  DummyAprilTag()
  : Node("dummy_apriltag")
  {
    parent_frame_ = declare_parameter("parent_frame", "base_link");
    tag_frame_ = declare_parameter("tag_frame", "tag0");
    detections_topic_ = declare_parameter("detections_topic", "/detections");
    family_ = declare_parameter("family", "tag36h11");
    tag_id_ = declare_parameter("tag_id", 0);
    decision_margin_ = declare_parameter("decision_margin", 100.0);
    x_ = declare_parameter("x", 0.55);
    y_ = declare_parameter("y", 0.0);
    z_ = declare_parameter("z", 0.85);
    yaw_ = declare_parameter("yaw", 0.0);
    const auto replay_box_pose = declare_parameter<std::vector<double>>(
      "replay_box_pose", std::vector<double>{});
    const auto replay_box_dimensions = declare_parameter<std::vector<double>>(
      "replay_box_dimensions", std::vector<double>{});
    const double replay_tag_to_box_yaw = declare_parameter("replay_tag_to_box_yaw", 0.0);
    const double publish_rate = declare_parameter("publish_rate", 30.0);

    if (parent_frame_.empty() || tag_frame_.empty() || detections_topic_.empty()) {
      throw std::invalid_argument("dummy AprilTag frame and topic names must not be empty");
    }
    if (parent_frame_ == tag_frame_) {
      throw std::invalid_argument("dummy AprilTag parent_frame and tag_frame must differ");
    }
    if (tag_id_ < 0 || !std::isfinite(decision_margin_) || decision_margin_ < 0.0 ||
      !std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(z_) ||
      !std::isfinite(yaw_) || !std::isfinite(publish_rate) || publish_rate <= 0.0)
    {
      throw std::invalid_argument("dummy AprilTag parameters must be finite and valid");
    }
    if (replay_box_pose.empty() != replay_box_dimensions.empty()) {
      throw std::invalid_argument(
              "replay_box_pose and replay_box_dimensions must be supplied together");
    }
    if (!replay_box_pose.empty()) {
      if (replay_box_pose.size() != 7U || replay_box_dimensions.size() != 3U ||
        !std::isfinite(replay_tag_to_box_yaw))
      {
        throw std::invalid_argument(
                "recorded box replay requires pose [x,y,z,qx,qy,qz,qw], dimensions [x,y,z], "
                "and a finite tag yaw");
      }
      const BoxDimensions dimensions{
        replay_box_dimensions[0], replay_box_dimensions[1], replay_box_dimensions[2]};
      if (dimensions.length <= 0.0 || dimensions.width <= 0.0 || dimensions.height <= 0.0) {
        throw std::invalid_argument("recorded box replay dimensions must be positive");
      }
      Eigen::Quaterniond box_rotation(
        replay_box_pose[6], replay_box_pose[3], replay_box_pose[4], replay_box_pose[5]);
      if (!box_rotation.coeffs().allFinite() || box_rotation.norm() < 1e-9 ||
        !std::isfinite(replay_box_pose[0]) || !std::isfinite(replay_box_pose[1]) ||
        !std::isfinite(replay_box_pose[2]))
      {
        throw std::invalid_argument("recorded box replay pose must be finite with a valid quaternion");
      }
      Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
      box_pose.translation() = Eigen::Vector3d(
        replay_box_pose[0], replay_box_pose[1], replay_box_pose[2]);
      box_pose.linear() = box_rotation.normalized().toRotationMatrix();
      Eigen::Isometry3d tag_to_box = Eigen::Isometry3d::Identity();
      tag_to_box.linear() = Eigen::AngleAxisd(
        replay_tag_to_box_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      tag_to_box.translation() = Eigen::Vector3d(0.0, 0.0, -dimensions.height / 2.0);
      const Eigen::Isometry3d tag_pose = box_pose * tag_to_box.inverse();
      x_ = tag_pose.translation().x();
      y_ = tag_pose.translation().y();
      z_ = tag_pose.translation().z();
      orientation_ = Eigen::Quaterniond(tag_pose.linear()).normalized();
    } else {
      orientation_ = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()));
    }

    detections_pub_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
      detections_topic_, rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(period, std::bind(&DummyAprilTag::publish, this));

    RCLCPP_WARN(
      get_logger(),
      "Publishing TEST-ONLY tag %d as %s -> %s at [%.3f, %.3f, %.3f]",
      tag_id_, parent_frame_.c_str(), tag_frame_.c_str(), x_, y_, z_);
  }

private:
  void publish()
  {
    const auto stamp = now();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = tag_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = z_;
    transform.transform.rotation.x = orientation_.x();
    transform.transform.rotation.y = orientation_.y();
    transform.transform.rotation.z = orientation_.z();
    transform.transform.rotation.w = orientation_.w();
    tf_broadcaster_->sendTransform(transform);

    apriltag_msgs::msg::AprilTagDetection detection;
    detection.family = family_;
    detection.id = tag_id_;
    detection.hamming = 0;
    detection.goodness = 1.0F;
    detection.decision_margin = static_cast<float>(decision_margin_);

    apriltag_msgs::msg::AprilTagDetectionArray detections;
    detections.header.stamp = stamp;
    detections.header.frame_id = parent_frame_;
    detections.detections.push_back(detection);
    detections_pub_->publish(detections);
  }

  std::string parent_frame_;
  std::string tag_frame_;
  std::string detections_topic_;
  std::string family_;
  int tag_id_;
  double decision_margin_;
  double x_;
  double y_;
  double z_;
  double yaw_;
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agibot_x2_manipulation::DummyAprilTag>());
  rclcpp::shutdown();
  return 0;
}
