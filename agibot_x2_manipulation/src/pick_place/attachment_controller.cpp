#include "pick_place/attachment_controller.hpp"

#include <chrono>
#include <exception>
#include <future>

namespace agibot_x2_manipulation
{

AttachmentController::AttachmentController(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config)
: node_(node), config_(config)
{
  attach_client_ = node_->create_client<std_srvs::srv::Trigger>(config_.attach_service);
  detach_client_ = node_->create_client<std_srvs::srv::Trigger>(config_.detach_service);
}

bool AttachmentController::attach(std::string & error, bool * request_dispatched)
{
  return call(attach_client_, error, request_dispatched);
}

bool AttachmentController::detach(std::string & error, bool * request_dispatched)
{
  return call(detach_client_, error, request_dispatched);
}

bool AttachmentController::simulated() const
{
  return config_.simulate_attachment;
}

bool AttachmentController::expected() const
{
  return expected_.load();
}

void AttachmentController::setExpected(bool expected)
{
  expected_.store(expected);
}

bool AttachmentController::call(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  std::string & error, bool * request_dispatched)
{
  if (request_dispatched) {
    *request_dispatched = false;
  }
  if (!config_.simulate_attachment) {
    return true;
  }
  try {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      error = "MuJoCo attachment service is unavailable";
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (request_dispatched) {
      *request_dispatched = true;
    }
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      error = "MuJoCo attachment service timed out";
      return false;
    }
    const auto response = future.get();
    error = response->message;
    return response->success;
  } catch (const std::exception & exception) {
    error = "MuJoCo attachment service failed: " + std::string(exception.what());
    return false;
  }
}

}  // namespace agibot_x2_manipulation
