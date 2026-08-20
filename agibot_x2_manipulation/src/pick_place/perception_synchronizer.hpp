#pragma once

#include "agibot_x2_manipulation/perception_readiness.hpp"
#include "pick_place/pick_place_config.hpp"
#include "pick_place/planning_scene_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <condition_variable>
#include <mutex>
#include <string>

namespace agibot_x2_manipulation
{

class PerceptionSynchronizer
{
public:
  PerceptionSynchronizer(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    PlanningSceneManager & planning_scene);

  bool clear(std::string & error);
  bool refresh(std::string & error);

private:
  template<typename MessageT>
  void recordSample(const MessageT & message, PerceptionSample & sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++sample.count;
    sample.header_nanoseconds = rclcpp::Time(message.header.stamp).nanoseconds();
    sample.receipt_nanoseconds = node_->now().nanoseconds();
    condition_.notify_all();
  }

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  PlanningSceneManager & planning_scene_;
  std::mutex mutex_;
  std::condition_variable condition_;
  PerceptionSample depth_sample_;
  PerceptionSample lidar_sample_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
};

}  // namespace agibot_x2_manipulation
