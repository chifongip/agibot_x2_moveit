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

    detections_pub_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
      detections_topic_, rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(period, std::bind(&DummyAprilTag::publish, this));

    RCLCPP_WARN(
      get_logger(),
      "Publishing TEST-ONLY tag %d as %s -> %s at [%.3f, %.3f, %.3f], yaw %.3f rad",
      tag_id_, parent_frame_.c_str(), tag_frame_.c_str(), x_, y_, z_, yaw_);
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
    transform.transform.rotation.z = std::sin(yaw_ / 2.0);
    transform.transform.rotation.w = std::cos(yaw_ / 2.0);
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
