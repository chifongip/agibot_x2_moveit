#pragma once

#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace agibot_x2_ros2_control
{

class X2SystemHardware final : public hardware_interface::SystemInterface
{
public:
  X2SystemHardware() = default;
  ~X2SystemHardware() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void state_callback(const aimdk_msgs::msg::JointStateArray::SharedPtr message);
  bool state_is_fresh_locked(SteadyTime now) const;
  bool parse_configuration();
  bool configure_ros_transport();
  bool configure_zmq_transport();
  void stop_io();
  void publish_ros_commands(const std::vector<double> & targets, bool damping_only);
  bool publish_zmq_commands(const std::vector<double> & targets);
  void enter_fault(const std::string & reason);
  std::vector<double> safe_targets_locked() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_running_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> fault_latched_{false};

  std::array<rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr,
    4> state_subscriptions_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_command_publisher_;

  mutable std::mutex data_mutex_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> commands_;
  std::vector<SteadyTime> update_times_;
  std::vector<bool> received_;
  std::vector<bool> claimed_;
  std::vector<std::size_t> arm_indices_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;

  std::string command_transport_{"ros_topic"};
  std::array<std::string, 4> state_topics_{};
  std::string arm_command_topic_;
  std::string zmq_endpoint_;
  double state_timeout_sec_{0.1};
  double activation_timeout_sec_{2.0};
  double fault_damping_{5.0};
  double fault_publish_duration_sec_{0.2};
  double zmq_publish_rate_hz_{50.0};
  SteadyTime last_zmq_publish_{};
  std::uint32_t command_sequence_{0};

  void * zmq_context_{nullptr};
  void * zmq_socket_{nullptr};
};

}  // namespace agibot_x2_ros2_control
