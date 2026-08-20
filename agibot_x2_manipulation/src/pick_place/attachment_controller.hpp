#pragma once

#include "pick_place/pick_place_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <string>

namespace agibot_x2_manipulation
{

class AttachmentController
{
public:
  AttachmentController(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config);

  bool attach(std::string & error, bool * request_dispatched = nullptr);
  bool detach(std::string & error, bool * request_dispatched = nullptr);
  bool simulated() const;
  bool expected() const;
  void setExpected(bool expected);

private:
  bool call(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    std::string & error, bool * request_dispatched);

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr attach_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detach_client_;
  std::atomic<bool> expected_{false};
};

}  // namespace agibot_x2_manipulation
