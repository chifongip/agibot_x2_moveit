#pragma once

#include "pick_place/pick_place_config.hpp"

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace agibot_x2_manipulation
{

/// Publishes RoboJuDo locomanipulation posture commands independently from
/// the arm-controller ZMQ transport. ZMQ is one-way, so readiness comes from
/// fresh direct HAL leg and waist measurements rather than a false receipt ACK.
class LocomanipulationPostureController
{
public:
  using CancelFunction = std::function<bool()>;

  struct Target
  {
    double height{0.0};
    double waist_yaw{0.0};
  };

  LocomanipulationPostureController(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config);
  ~LocomanipulationPostureController();

  LocomanipulationPostureController(const LocomanipulationPostureController &) = delete;
  LocomanipulationPostureController & operator=(const LocomanipulationPostureController &) = delete;

  bool enabled() const;
  bool targetActive() const;
  Target target() const;
  std::string endpoint() const;

  /// Start continuously publishing a validated posture target. Until this is
  /// called, the publisher remains idle and sends no command at startup.
  bool setTarget(const Target & target, std::string & error);

  /// Stop continuously publishing this source's target. RoboJuDo intentionally
  /// holds its last accepted posture after its source lease expires, so this is
  /// an authority handoff, not a physical reset or a safe-pose command.
  bool deactivateTarget();

  /// Set the continuously published posture target, then require fresh leg
  /// and waist feedback plus the configured settling interval. This does not
  /// prove receipt by RoboJuDo, which has no acknowledgement channel.
  bool applyAndWait(
    const Target & target, const CancelFunction & canceled, std::string & error);

  /// Require a new lower-body feedback window while retaining the current
  /// continuously published target.
  bool waitForFreshFeedback(const CancelFunction & canceled, std::string & error);

private:
  struct FeedbackSnapshot
  {
    uint64_t leg_generation{0};
    uint64_t waist_generation{0};
  };

  void publish();
  void onLegState(const aimdk_msgs::msg::JointStateArray::SharedPtr message);
  void onWaistState(const aimdk_msgs::msg::JointStateArray::SharedPtr message);
  bool waitForFreshFeedback(
    FeedbackSnapshot snapshot, const std::chrono::steady_clock::time_point & not_before,
    const CancelFunction & canceled, std::string & error);
  FeedbackSnapshot feedbackSnapshot() const;

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  mutable std::mutex mutex_;
  std::condition_variable feedback_condition_;
  Target target_;
  bool target_active_{false};
  uint64_t leg_generation_{0};
  uint64_t waist_generation_{0};
  std::string bound_endpoint_;
  void * zmq_context_{nullptr};
  void * zmq_socket_{nullptr};
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr leg_subscription_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr waist_subscription_;
};

}  // namespace agibot_x2_manipulation
