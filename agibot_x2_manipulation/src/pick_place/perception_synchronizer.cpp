#include "pick_place/perception_synchronizer.hpp"

#include <chrono>
#include <exception>
#include <future>

namespace agibot_x2_manipulation
{

PerceptionSynchronizer::PerceptionSynchronizer(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
  PlanningSceneManager & planning_scene)
: node_(node), config_(config), planning_scene_(planning_scene)
{
  if (usesDepth(config_.perception_source)) {
    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      config_.depth_filtered_cloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr message) {
        recordSample(*message, depth_sample_);
      });
  }
  if (usesLidar(config_.perception_source)) {
    lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.lidar_filtered_cloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        recordSample(*message, lidar_sample_);
      });
  }
  clear_client_ = node_->create_client<std_srvs::srv::Empty>(config_.octomap_clear_service);
}

bool PerceptionSynchronizer::clear(std::string & error)
{
  const auto timeout = std::chrono::duration<double>(config_.octomap_refill_timeout);
  try {
    if (!clear_client_->wait_for_service(timeout)) {
      error = "OctoMap clear service is unavailable: " + config_.octomap_clear_service;
      return false;
    }
    auto future = clear_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>());
    if (future.wait_for(timeout) != std::future_status::ready) {
      error = "OctoMap clear service timed out: " + config_.octomap_clear_service;
      return false;
    }
    future.get();
  } catch (const std::exception & exception) {
    error = "OctoMap clear request failed: " + std::string(exception.what());
    return false;
  }
  return true;
}

bool PerceptionSynchronizer::refresh(std::string & error)
{
  if (config_.perception_source == Perception3dSource::NONE) {
    return true;
  }
  if (!clear(error)) {
    return false;
  }

  const auto timeout = std::chrono::duration<double>(config_.octomap_refill_timeout);
  std::unique_lock<std::mutex> lock(mutex_);
  const PerceptionSnapshot snapshot{depth_sample_.count, lidar_sample_.count};
  const auto ready = [this, &snapshot]() {
      return perceptionReady(
        config_.perception_source, depth_sample_, lidar_sample_, snapshot,
        static_cast<uint64_t>(config_.octomap_post_clear_samples), node_->now().nanoseconds(),
        static_cast<int64_t>(config_.maximum_filtered_cloud_age * 1e9),
        static_cast<int64_t>(config_.maximum_filtered_cloud_future_skew * 1e9));
    };
  if (!condition_.wait_for(lock, timeout, ready)) {
    std::string sources;
    if (usesDepth(config_.perception_source)) {
      sources += "depth";
    }
    if (usesLidar(config_.perception_source)) {
      sources += sources.empty() ? "lidar" : " and lidar";
    }
    error = "no fresh post-clear filtered cloud samples from " + sources +
      " (check topic remapping, timestamps, CameraInfo, and sensor-to-base_link TF)";
    return false;
  }
  lock.unlock();

  rclcpp::sleep_for(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.octomap_processing_delay)));
  if (!planning_scene_.synchronize(error)) {
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "OctoMap cleared and refilled with %d post-clear sample(s)",
    config_.octomap_post_clear_samples);
  return true;
}

}  // namespace agibot_x2_manipulation
