#include "pick_place/locomanipulation_posture_controller.hpp"

#include <zmq.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

bool validFeedback(const aimdk_msgs::msg::JointStateArray & message)
{
  if (message.joints.empty()) {
    return false;
  }
  for (const auto & joint : message.joints) {
    if (joint.name.empty() || !std::isfinite(joint.position) ||
      !std::isfinite(joint.velocity))
    {
      return false;
    }
  }
  return true;
}

bool validTarget(const LocomanipulationPostureController::Target & target)
{
  return std::isfinite(target.height) && target.height >= 0.30 && target.height <= 0.64 &&
         std::isfinite(target.waist_yaw) && target.waist_yaw >= -1.5708 &&
         target.waist_yaw <= 1.5708;
}

}  // namespace

LocomanipulationPostureController::LocomanipulationPostureController(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config)
: node_(node), config_(config)
{
  if (!config_.posture_zmq_enabled) {
    RCLCPP_INFO(node_->get_logger(), "Locomanipulation posture ZMQ control is disabled");
    return;
  }

  zmq_context_ = zmq_ctx_new();
  if (!zmq_context_) {
    throw std::runtime_error(
            "failed to create locomanipulation posture ZMQ context: " +
            std::string(zmq_strerror(errno)));
  }
  zmq_socket_ = zmq_socket(zmq_context_, ZMQ_PUB);
  if (!zmq_socket_) {
    const std::string error = zmq_strerror(errno);
    zmq_ctx_term(zmq_context_);
    zmq_context_ = nullptr;
    throw std::runtime_error("failed to create locomanipulation posture ZMQ PUB socket: " + error);
  }
  const int linger = 0;
  const int high_water_mark = 1;
  (void)zmq_setsockopt(zmq_socket_, ZMQ_LINGER, &linger, sizeof(linger));
  (void)zmq_setsockopt(zmq_socket_, ZMQ_SNDHWM, &high_water_mark, sizeof(high_water_mark));
  if (zmq_bind(zmq_socket_, config_.posture_zmq_endpoint.c_str()) != 0) {
    const std::string error = zmq_strerror(errno);
    zmq_close(zmq_socket_);
    zmq_socket_ = nullptr;
    zmq_ctx_term(zmq_context_);
    zmq_context_ = nullptr;
    throw std::runtime_error(
            "failed to bind locomanipulation posture ZMQ endpoint '" +
            config_.posture_zmq_endpoint + "': " + error);
  }
  char endpoint[256] = {};
  std::size_t endpoint_size = sizeof(endpoint);
  if (zmq_getsockopt(zmq_socket_, ZMQ_LAST_ENDPOINT, endpoint, &endpoint_size) == 0) {
    bound_endpoint_ = endpoint;
  } else {
    bound_endpoint_ = config_.posture_zmq_endpoint;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / config_.posture_zmq_publish_rate_hz));
  publish_timer_ = node_->create_wall_timer(period, [this]() {publish();});
  leg_subscription_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
    config_.leg_state_topic, rclcpp::SensorDataQoS(),
    [this](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {onLegState(message);});
  waist_subscription_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
    config_.waist_state_topic, rclcpp::SensorDataQoS(),
    [this](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {onWaistState(message);});
  RCLCPP_INFO(
    node_->get_logger(),
    "Locomanipulation posture ZMQ publisher bound to %s at %.1f Hz; waiting for an "
    "explicit posture request",
    bound_endpoint_.c_str(), config_.posture_zmq_publish_rate_hz);
}

LocomanipulationPostureController::~LocomanipulationPostureController()
{
  publish_timer_.reset();
  leg_subscription_.reset();
  waist_subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  if (zmq_socket_) {
    zmq_close(zmq_socket_);
    zmq_socket_ = nullptr;
  }
  if (zmq_context_) {
    zmq_ctx_term(zmq_context_);
    zmq_context_ = nullptr;
  }
}

bool LocomanipulationPostureController::enabled() const
{
  return config_.posture_zmq_enabled;
}

bool LocomanipulationPostureController::targetActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_active_;
}

LocomanipulationPostureController::Target LocomanipulationPostureController::target() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_;
}

std::string LocomanipulationPostureController::endpoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bound_endpoint_;
}

bool LocomanipulationPostureController::setTarget(
  const Target & target, std::string & error)
{
  if (!enabled()) {
    error = "locomanipulation posture ZMQ control is disabled";
    return false;
  }
  if (!validTarget(target)) {
    error = "requested locomanipulation posture is outside RoboJuDo's trained command range";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = target;
    target_active_ = true;
  }
  // Send once synchronously so callers can start a feedback window only after
  // the command has reached the local ZMQ PUB transport. The timer keeps it
  // alive afterwards for RoboJuDo's stale-command timeout.
  publish();
  return true;
}

bool LocomanipulationPostureController::deactivateTarget()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_active = target_active_;
  target_active_ = false;
  return was_active;
}

bool LocomanipulationPostureController::applyAndWait(
  const Target & target, const CancelFunction & canceled, std::string & error)
{
  if (!setTarget(target, error)) {
    return false;
  }
  const FeedbackSnapshot snapshot = feedbackSnapshot();
  const auto not_before = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.posture_settle_duration));
  return waitForFreshFeedback(snapshot, not_before, canceled, error);
}

bool LocomanipulationPostureController::waitForFreshFeedback(
  const CancelFunction & canceled, std::string & error)
{
  if (!enabled()) {
    error = "locomanipulation posture ZMQ control is disabled";
    return false;
  }
  const auto not_before = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.posture_settle_duration));
  return waitForFreshFeedback(feedbackSnapshot(), not_before, canceled, error);
}

void LocomanipulationPostureController::publish()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!zmq_socket_ || !target_active_) {
    return;
  }
  std::ostringstream payload;
  payload.imbue(std::locale::classic());
  payload << std::setprecision(17) << "{\"height\":" << target_.height <<
    ",\"waist_yaw\":" << target_.waist_yaw << "}";
  const std::string message = payload.str();
  if (zmq_send(zmq_socket_, message.data(), message.size(), ZMQ_DONTWAIT) < 0 &&
    errno != EAGAIN)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Locomanipulation posture ZMQ send failed: %s", zmq_strerror(errno));
  }
}

void LocomanipulationPostureController::onLegState(
  const aimdk_msgs::msg::JointStateArray::SharedPtr message)
{
  if (!validFeedback(*message)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Ignoring invalid locomanipulation leg feedback from %s",
      config_.leg_state_topic.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++leg_generation_;
  }
  feedback_condition_.notify_all();
}

void LocomanipulationPostureController::onWaistState(
  const aimdk_msgs::msg::JointStateArray::SharedPtr message)
{
  if (!validFeedback(*message)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Ignoring invalid locomanipulation waist feedback from %s",
      config_.waist_state_topic.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++waist_generation_;
  }
  feedback_condition_.notify_all();
}

LocomanipulationPostureController::FeedbackSnapshot
LocomanipulationPostureController::feedbackSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return FeedbackSnapshot{leg_generation_, waist_generation_};
}

bool LocomanipulationPostureController::waitForFreshFeedback(
  FeedbackSnapshot snapshot, const std::chrono::steady_clock::time_point & not_before,
  const CancelFunction & canceled, std::string & error)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.posture_settle_timeout));
  std::unique_lock<std::mutex> lock(mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (canceled && canceled()) {
      error = "locomanipulation posture wait canceled";
      return false;
    }
    const bool samples_ready =
      leg_generation_ >= snapshot.leg_generation + config_.posture_settle_samples &&
      waist_generation_ >= snapshot.waist_generation + config_.posture_settle_samples;
    if (samples_ready && std::chrono::steady_clock::now() >= not_before) {
      return true;
    }
    feedback_condition_.wait_for(lock, std::chrono::milliseconds(20));
  }
  std::ostringstream message;
  message << "timed out waiting for " << config_.posture_settle_samples <<
    " fresh direct leg and waist feedback samples after locomanipulation posture command "
    "(leg=" << (leg_generation_ - snapshot.leg_generation) << ", waist=" <<
    (waist_generation_ - snapshot.waist_generation) << ", timeout=" <<
    config_.posture_settle_timeout << " s)";
  error = message.str();
  return false;
}

}  // namespace agibot_x2_manipulation
