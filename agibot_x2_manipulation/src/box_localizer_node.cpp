#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/box_profile_registry.hpp"

#include <agibot_x2_manipulation_msgs/msg/box_state.hpp>
#include <agibot_x2_manipulation_msgs/msg/box_state_array.hpp>
#include <agibot_x2_manipulation_msgs/srv/reload_box_profiles.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace agibot_x2_manipulation {
namespace {

using ReloadBoxProfiles =
  agibot_x2_manipulation_msgs::srv::ReloadBoxProfiles;

BoxProfile legacyProfile(const BoxDimensions &dimensions, double tag_to_box_yaw,
                         const Eigen::Vector3d &tag_to_box_offset, int tag_id) {
  BoxProfile profile;
  profile.id = "legacy";
  profile.dimensions = dimensions;
  profile.tag_to_box_yaw = tag_to_box_yaw;
  profile.tag_to_box_offset = tag_to_box_offset;
  profile.tag_to_box_center = topTagToBoxCenter(
    dimensions, tag_to_box_yaw, tag_to_box_offset);
  profile.tag_ids = {tag_id};
  return profile;
}

template <typename T>
T parameter(rclcpp::Node &node, const std::string &name,
            const T &default_value) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

} // namespace

class BoxLocalizer : public rclcpp::Node {
public:
  explicit BoxLocalizer(const rclcpp::NodeOptions &options)
      : Node("box_localizer", options), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    planning_frame_ =
        parameter<std::string>(*this, "planning_frame", "base_link");
    const auto topic =
        parameter<std::string>(*this, "detections_topic", "/detections");
    legacy_tag_frame_ = parameter<std::string>(*this, "tag_frame", "tag0");
    legacy_tag_id_ = parameter<int>(*this, "tag_id", 0);
    const auto dimensions = parameter<std::vector<double>>(
        *this, "box_dimensions", {0.30, 0.20, 0.15});
    if (dimensions.size() != 3U) {
      throw std::runtime_error(
          "box_dimensions must contain [length, width, height]");
    }
    const BoxDimensions legacy_dimensions{dimensions[0], dimensions[1],
                                          dimensions[2]};
    const double legacy_tag_to_box_yaw =
        parameter<double>(*this, "tag_to_box_yaw", 0.0);
    const auto legacy_offset = parameter<std::vector<double>>(
        *this, "tag_to_box_offset", {0.0, 0.0, 0.0});
    if (legacy_offset.size() != 3U) {
      throw std::runtime_error("tag_to_box_offset must contain [x, y, z]");
    }
    const Eigen::Vector3d legacy_tag_to_box_offset(
        legacy_offset[0], legacy_offset[1], legacy_offset[2]);
    if (!std::isfinite(legacy_tag_to_box_yaw) ||
        !legacy_tag_to_box_offset.allFinite()) {
      throw std::runtime_error("top-tag calibration values must be finite");
    }

    profiles_ = BoxProfileRegistry::fromParameters(*this);
    legacy_mode_ = profiles_.empty();
    if (legacy_mode_) {
      legacy_profile_ = legacyProfile(legacy_dimensions, legacy_tag_to_box_yaw,
                                      legacy_tag_to_box_offset, legacy_tag_id_);
    }

    stable_count_ = static_cast<std::size_t>(
        parameter<int>(*this, "stable_sample_count", 10));
    max_age_ = parameter<double>(*this, "maximum_pose_age", 0.25);
    max_position_spread_ =
        parameter<double>(*this, "maximum_position_spread", 0.005);
    max_angular_spread_ =
        parameter<double>(*this, "maximum_angular_spread", 0.0523598776);
    max_box_tilt_ = parameter<double>(*this, "maximum_box_tilt", 0.0872664626);
    minimum_margin_ = parameter<double>(*this, "minimum_decision_margin", 20.0);
    if (stable_count_ < 2U) {
      throw std::runtime_error("stable_sample_count must be at least 2");
    }

    const auto states_topic =
        parameter<std::string>(*this, "box_states_topic", "/box_states");
    state_pub_ =
        create_publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>(
            states_topic, 10);
    // Preserve the single-box topic when no profile catalog was supplied.
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/box_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/box_markers", 10);
    detections_sub_ =
        create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&BoxLocalizer::onDetections, this,
                      std::placeholders::_1));
    reload_profiles_service_ = create_service<ReloadBoxProfiles>(
      "~/reload_box_profiles",
      std::bind(&BoxLocalizer::reloadProfiles, this, std::placeholders::_1,
      std::placeholders::_2));
  }

private:
  void reloadProfiles(
    const std::shared_ptr<ReloadBoxProfiles::Request> request,
    std::shared_ptr<ReloadBoxProfiles::Response> response)
  {
    try {
      auto candidate = BoxProfileRegistry::fromYamlFile(request->profiles_file);
      if (candidate.empty()) {
        response->message = "box-profile catalog must contain at least one profile";
        response->profile_version = profile_version_;
        return;
      }
      if (!request->dry_run) {
        profiles_ = std::move(candidate);
        legacy_mode_ = false;
        samples_.clear();
        ++profile_version_;
      }
      response->success = true;
      response->profile_version = profile_version_;
      response->message = request->dry_run ?
        "box-profile catalog is valid" : "box-profile catalog reloaded";
    } catch (const std::exception & error) {
      response->message = error.what();
      response->profile_version = profile_version_;
    }
  }

  const BoxProfile *profileForTag(int tag_id) const {
    if (!legacy_mode_) {
      return profiles_.profileForTag(tag_id);
    }
    return tag_id == legacy_tag_id_ ? &legacy_profile_ : nullptr;
  }

  std::string tagFrame(int tag_id) const {
    return legacy_mode_ ? legacy_tag_frame_ : profiles_.tagFrame(tag_id);
  }

  std::string instanceId(int tag_id) const {
    return legacy_mode_ ? "legacy" : profiles_.instanceId(tag_id);
  }

  void onDetections(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message) {
    for (const auto &detection : message->detections) {
      const BoxProfile *profile = profileForTag(detection.id);
      if (!profile || detection.decision_margin < minimum_margin_) {
        continue;
      }
      localize(detection.id, *profile);
    }
  }

  void localize(int tag_id, const BoxProfile &profile) {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          planning_frame_, tagFrame(tag_id), tf2::TimePointZero);
      const rclcpp::Time transform_stamp(transform.header.stamp);
      if ((now() - transform_stamp).seconds() > max_age_) {
        return;
      }
      const Eigen::Isometry3d box_pose = boxPoseFromTag(
          tf2::transformToEigen(transform), profile.tag_to_box_center);
      const Eigen::Vector3d box_up =
          box_pose.linear() * Eigen::Vector3d::UnitZ();
      const double tilt = std::acos(
          std::clamp(box_up.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
      if (tilt > max_box_tilt_) {
        samples_[tag_id].clear();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Box tag %d tilt %.1f deg exceeds upright limit",
                             tag_id, tilt * 180.0 / 3.14159265358979323846);
        return;
      }

      auto &samples = samples_[tag_id];
      samples.push_back(box_pose);
      while (samples.size() > stable_count_) {
        samples.pop_front();
      }
      if (samples.size() == stable_count_) {
        publishIfStable(tag_id, profile, samples, transform.header.stamp);
      }
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Tag %d TF unavailable: %s", tag_id, error.what());
    }
  }

  void publishIfStable(int tag_id, const BoxProfile &profile,
                       const std::deque<Eigen::Isometry3d> &samples,
                       const builtin_interfaces::msg::Time &stamp) {
    Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
    Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
    const Eigen::Quaterniond reference(samples.front().linear());
    for (const auto &sample : samples) {
      mean_position += sample.translation();
      Eigen::Quaterniond quaternion(sample.linear());
      if (quaternion.dot(reference) < 0.0) {
        quaternion.coeffs() *= -1.0;
      }
      quaternion_sum += quaternion.coeffs();
    }
    mean_position /= static_cast<double>(samples.size());
    Eigen::Quaterniond mean_quaternion;
    mean_quaternion.coeffs() = quaternion_sum.normalized();

    double position_spread = 0.0;
    double angular_spread = 0.0;
    for (const auto &sample : samples) {
      position_spread = std::max(position_spread,
                                 (sample.translation() - mean_position).norm());
      const Eigen::Quaterniond quaternion(sample.linear());
      angular_spread = std::max(
          angular_spread,
          2.0 * std::acos(std::clamp(std::abs(quaternion.dot(mean_quaternion)),
                                     0.0, 1.0)));
    }
    if (position_spread > max_position_spread_ ||
        angular_spread > max_angular_spread_) {
      return;
    }

    agibot_x2_manipulation_msgs::msg::BoxState state;
    state.header.frame_id = planning_frame_;
    state.header.stamp = stamp;
    state.instance_id = instanceId(tag_id);
    state.profile_id = profile.id;
    state.pose.pose.position.x = mean_position.x();
    state.pose.pose.position.y = mean_position.y();
    state.pose.pose.position.z = mean_position.z();
    state.pose.pose.orientation = tf2::toMsg(mean_quaternion);
    const double position_variance = position_spread * position_spread;
    const double angular_variance = angular_spread * angular_spread;
    state.pose.covariance[0] = state.pose.covariance[7] =
        state.pose.covariance[14] = position_variance;
    state.pose.covariance[21] = state.pose.covariance[28] =
        state.pose.covariance[35] = angular_variance;

    agibot_x2_manipulation_msgs::msg::BoxStateArray states;
    states.header = state.header;
    states.boxes.push_back(state);
    state_pub_->publish(states);

    if (legacy_mode_) {
      geometry_msgs::msg::PoseWithCovarianceStamped legacy_pose;
      legacy_pose.header = state.header;
      legacy_pose.pose = state.pose;
      pose_pub_->publish(legacy_pose);
    }

    visualization_msgs::msg::Marker marker;
    marker.header = state.header;
    marker.ns = "localized_box";
    marker.id = tag_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = state.pose.pose;
    marker.scale.x = profile.dimensions.length;
    marker.scale.y = profile.dimensions.width;
    marker.scale.z = profile.dimensions.height;
    marker.color.r = 0.2F;
    marker.color.g = 0.7F;
    marker.color.b = 0.9F;
    marker.color.a = 0.35F;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  std::string planning_frame_;
  std::string legacy_tag_frame_;
  int legacy_tag_id_{0};
  BoxProfileRegistry profiles_;
  bool legacy_mode_{true};
  BoxProfile legacy_profile_;
  std::size_t stable_count_{0};
  double max_age_{0.0};
  double max_position_spread_{0.0};
  double max_angular_spread_{0.0};
  double max_box_tilt_{0.0};
  double minimum_margin_{0.0};
  uint64_t profile_version_{0};
  std::map<int, std::deque<Eigen::Isometry3d>> samples_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      detections_sub_;
  rclcpp::Service<ReloadBoxProfiles>::SharedPtr reload_profiles_service_;
  rclcpp::Publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>::SharedPtr
      state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
};

} // namespace agibot_x2_manipulation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true);
  rclcpp::spin(std::make_shared<agibot_x2_manipulation::BoxLocalizer>(options));
  rclcpp::shutdown();
  return 0;
}
