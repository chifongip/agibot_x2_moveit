#include "agibot_x2_ros2_control/x2_system_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <zmq.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace agibot_x2_ros2_control
{
namespace
{
constexpr std::array<const char *, 31> kExpectedJoints = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
  "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
  "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
  "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_pitch_joint",
  "waist_roll_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_yaw_joint",
  "left_wrist_pitch_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
  "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
  "head_yaw_joint", "head_pitch_joint"};

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
      return interface.name == name;
    });
}

double parse_positive(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  double fallback, bool allow_zero = false)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return fallback;
  }
  std::size_t consumed = 0;
  const double value = std::stod(it->second, &consumed);
  if (consumed != it->second.size() || !std::isfinite(value) ||
    (allow_zero ? value < 0.0 : value <= 0.0))
  {
    throw std::invalid_argument("invalid hardware parameter '" + name + "': " + it->second);
  }
  return value;
}

std::string get_parameter(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  const std::string & fallback)
{
  const auto it = parameters.find(name);
  return it == parameters.end() ? fallback : it->second;
}
}  // namespace

X2SystemHardware::~X2SystemHardware()
{
  active_ = false;
  stop_io();
}

hardware_interface::CallbackReturn X2SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    if (!parse_configuration()) {
      return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("agibot_x2_system_hardware");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    const auto qos = rclcpp::SensorDataQoS();
    for (std::size_t i = 0; i < state_topics_.size(); ++i) {
      state_subscriptions_[i] = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        state_topics_[i], qos,
        [this](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {
          state_callback(message);
        });
    }

    if (command_transport_ == "ros_topic") {
      if (!configure_ros_transport()) {
        return CallbackReturn::ERROR;
      }
    } else if (!configure_zmq_transport()) {
      return CallbackReturn::ERROR;
    }

    executor_->add_node(node_);
    spin_running_ = true;
    spin_thread_ = std::thread(
      [this]() {
        while (spin_running_ && rclcpp::ok()) {
          executor_->spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("X2SystemHardware"), "Initialization failed: %s",
      exception.what());
    stop_io();
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Configured X2 hardware with '%s' command transport",
    command_transport_.c_str());
  return CallbackReturn::SUCCESS;
}

bool X2SystemHardware::parse_configuration()
{
  const std::set<std::string> expected(kExpectedJoints.begin(), kExpectedJoints.end());
  std::set<std::string> configured;
  if (info_.joints.size() != expected.size()) {
    throw std::invalid_argument("the X2 hardware must define exactly 31 joints");
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  positions_.assign(info_.joints.size(), nan);
  velocities_.assign(info_.joints.size(), nan);
  efforts_.assign(info_.joints.size(), nan);
  commands_.assign(info_.joints.size(), nan);
  update_times_.resize(info_.joints.size());
  received_.assign(info_.joints.size(), false);
  claimed_.assign(info_.joints.size(), false);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    if (!configured.insert(joint.name).second || expected.count(joint.name) == 0) {
      throw std::invalid_argument("unexpected or duplicate X2 joint: " + joint.name);
    }
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      throw std::invalid_argument(
              "joint must export position, velocity, and effort: " +
              joint.name);
    }
    joint_index_[joint.name] = i;
    const bool has_position_command =
      has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION);
    const bool is_arm = joint.name.rfind("left_shoulder_", 0) == 0 ||
      joint.name.rfind("right_shoulder_", 0) == 0 || joint.name == "left_elbow_joint" ||
      joint.name == "right_elbow_joint" || joint.name.rfind("left_wrist_", 0) == 0 ||
      joint.name.rfind("right_wrist_", 0) == 0;
    if (has_position_command != is_arm || joint.command_interfaces.size() != (is_arm ? 1u : 0u)) {
      throw std::invalid_argument(
              "only the 14 arm joints may export a position command: " + joint.name);
    }
    if (is_arm) {
      arm_indices_.push_back(i);
      stiffness_.push_back(parse_positive(joint.parameters, "stiffness", 0.0));
      damping_.push_back(parse_positive(joint.parameters, "damping", 0.0));
    }
  }

  const auto & parameters = info_.hardware_parameters;
  command_transport_ = get_parameter(parameters, "command_transport", "ros_topic");
  if (command_transport_ != "ros_topic" && command_transport_ != "zmq") {
    throw std::invalid_argument("command_transport must be 'ros_topic' or 'zmq'");
  }
  state_topics_ = {
    get_parameter(parameters, "leg_state_topic", "/aima/hal/joint/leg/state"),
    get_parameter(parameters, "waist_state_topic", "/aima/hal/joint/waist/state"),
    get_parameter(parameters, "arm_state_topic", "/aima/hal/joint/arm/state"),
    get_parameter(parameters, "head_state_topic", "/aima/hal/joint/head/state")};
  arm_command_topic_ =
    get_parameter(parameters, "arm_command_topic", "/aima/hal/joint/arm/command");
  zmq_endpoint_ = get_parameter(parameters, "zmq_endpoint", "tcp://*:8559");
  state_timeout_sec_ = parse_positive(parameters, "state_timeout_sec", 0.1);
  activation_timeout_sec_ = parse_positive(parameters, "activation_timeout_sec", 2.0);
  fault_damping_ = parse_positive(parameters, "fault_damping", 5.0, true);
  fault_publish_duration_sec_ =
    parse_positive(parameters, "fault_publish_duration_sec", 0.2, true);
  zmq_publish_rate_hz_ = parse_positive(parameters, "zmq_publish_rate_hz", 50.0);
  return arm_indices_.size() == 14;
}

bool X2SystemHardware::configure_ros_transport()
{
  arm_command_publisher_ = node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(
    arm_command_topic_, rclcpp::SensorDataQoS());
  return static_cast<bool>(arm_command_publisher_);
}

bool X2SystemHardware::configure_zmq_transport()
{
  zmq_context_ = zmq_ctx_new();
  if (!zmq_context_) {
    RCLCPP_ERROR(node_->get_logger(), "Could not create ZMQ context: %s", zmq_strerror(errno));
    return false;
  }
  zmq_socket_ = zmq_socket(zmq_context_, ZMQ_PUB);
  if (!zmq_socket_) {
    RCLCPP_ERROR(node_->get_logger(), "Could not create ZMQ PUB socket: %s", zmq_strerror(errno));
    return false;
  }
  const int linger = 0;
  const int high_water_mark = 1;
  zmq_setsockopt(zmq_socket_, ZMQ_LINGER, &linger, sizeof(linger));
  zmq_setsockopt(zmq_socket_, ZMQ_SNDHWM, &high_water_mark, sizeof(high_water_mark));
  if (zmq_bind(zmq_socket_, zmq_endpoint_.c_str()) != 0) {
    RCLCPP_ERROR(
      node_->get_logger(), "Could not bind ZMQ endpoint %s: %s", zmq_endpoint_.c_str(),
      zmq_strerror(errno));
    return false;
  }
  return true;
}

void X2SystemHardware::stop_io()
{
  spin_running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  if (zmq_socket_) {
    zmq_close(zmq_socket_);
    zmq_socket_ = nullptr;
  }
  if (zmq_context_) {
    zmq_ctx_term(zmq_context_);
    zmq_context_ = nullptr;
  }
}

std::vector<hardware_interface::StateInterface> X2SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &velocities_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> X2SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(arm_indices_.size());
  for (const auto index : arm_indices_) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &commands_[index]);
  }
  return interfaces;
}

void X2SystemHardware::state_callback(
  const aimdk_msgs::msg::JointStateArray::SharedPtr message)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (const auto & joint : message->joints) {
    const auto found = joint_index_.find(joint.name);
    if (found == joint_index_.end()) {
      continue;
    }
    if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity) ||
      !std::isfinite(joint.effort))
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, "Ignoring non-finite state for %s",
        joint.name.c_str());
      continue;
    }
    const auto index = found->second;
    positions_[index] = joint.position;
    velocities_[index] = joint.velocity;
    efforts_[index] = joint.effort;
    received_[index] = true;
    update_times_[index] = now;
  }
}

bool X2SystemHardware::state_is_fresh_locked(SteadyTime now) const
{
  const auto timeout = std::chrono::duration<double>(state_timeout_sec_);
  for (std::size_t i = 0; i < received_.size(); ++i) {
    if (!received_[i] || now - update_times_[i] > timeout || !std::isfinite(positions_[i]) ||
      !std::isfinite(velocities_[i]) || !std::isfinite(efforts_[i]))
    {
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn X2SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  fault_latched_ = false;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(activation_timeout_sec_));
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (state_is_fresh_locked(std::chrono::steady_clock::now())) {
        for (const auto index : arm_indices_) {
          commands_[index] = positions_[index];
        }
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "X2 hardware activated at measured arm positions");
        return CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_ERROR(node_->get_logger(), "Activation rejected: not all 31 joint states are fresh");
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn X2SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (command_transport_ == "ros_topic") {
    enter_fault("hardware deactivated");
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type X2SystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto valid = [this](const std::string & interface) {
      const auto separator = interface.rfind('/');
      if (separator == std::string::npos ||
        interface.substr(separator + 1) != hardware_interface::HW_IF_POSITION)
      {
        return false;
      }
      const auto found = joint_index_.find(interface.substr(0, separator));
      return found != joint_index_.end() &&
             std::find(
        arm_indices_.begin(), arm_indices_.end(),
        found->second) != arm_indices_.end();
    };
  for (const auto & interface : start_interfaces) {
    if (interface.find('/') != std::string::npos && !valid(interface)) {
      continue;
    }
  }
  for (const auto & interface : stop_interfaces) {
    if (interface.find('/') != std::string::npos && !valid(interface)) {
      continue;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type X2SystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto update_claim = [this](const std::string & interface, bool claimed) {
      const auto separator = interface.rfind('/');
      if (separator == std::string::npos ||
        interface.substr(separator + 1) != hardware_interface::HW_IF_POSITION)
      {
        return;
      }
      const auto found = joint_index_.find(interface.substr(0, separator));
      if (found == joint_index_.end()) {
        return;
      }
      const auto index = found->second;
      if (std::find(arm_indices_.begin(), arm_indices_.end(), index) == arm_indices_.end()) {
        return;
      }
      claimed_[index] = claimed;
      commands_[index] = positions_[index];
    };
  for (const auto & interface : stop_interfaces) {
    update_claim(interface, false);
  }
  for (const auto & interface : start_interfaces) {
    update_claim(interface, true);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type X2SystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (state_is_fresh_locked(std::chrono::steady_clock::now())) {
      return hardware_interface::return_type::OK;
    }
  }
  enter_fault("joint state missing, stale, or non-finite");
  return hardware_interface::return_type::ERROR;
}

std::vector<double> X2SystemHardware::safe_targets_locked() const
{
  std::vector<double> targets;
  targets.reserve(arm_indices_.size());
  for (const auto index : arm_indices_) {
    const double target = claimed_[index] &&
      std::isfinite(commands_[index]) ? commands_[index] : positions_[index];
    targets.push_back(target);
  }
  return targets;
}

hardware_interface::return_type X2SystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_ || fault_latched_) {
    return hardware_interface::return_type::OK;
  }
  std::vector<double> targets;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    targets = safe_targets_locked();
  }
  if (std::any_of(
      targets.begin(), targets.end(),
      [](double value) {return !std::isfinite(value);}))
  {
    enter_fault("controller produced a non-finite arm target");
    return hardware_interface::return_type::ERROR;
  }
  if (command_transport_ == "ros_topic") {
    publish_ros_commands(targets, false);
  } else if (!publish_zmq_commands(targets)) {
    enter_fault("ZMQ command publication failed");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void X2SystemHardware::publish_ros_commands(
  const std::vector<double> & targets, bool damping_only)
{
  if (!arm_command_publisher_) {
    return;
  }
  aimdk_msgs::msg::JointCommandArray message;
  const auto stamp = node_->now();
  message.header.stamp = stamp;
  message.header.meas_stamp = stamp;
  message.header.sequence = command_sequence_++;
  message.joints.reserve(arm_indices_.size());
  for (std::size_t arm_index = 0; arm_index < arm_indices_.size(); ++arm_index) {
    aimdk_msgs::msg::JointCommand command;
    command.name = info_.joints[arm_indices_[arm_index]].name;
    command.position = damping_only ? 0.0 : targets[arm_index];
    command.velocity = 0.0;
    command.effort = 0.0;
    command.stiffness = damping_only ? 0.0 : stiffness_[arm_index];
    command.damping = damping_only ? fault_damping_ : damping_[arm_index];
    message.joints.push_back(std::move(command));
  }
  arm_command_publisher_->publish(message);
}

bool X2SystemHardware::publish_zmq_commands(const std::vector<double> & targets)
{
  const auto now = std::chrono::steady_clock::now();
  const auto interval = std::chrono::duration<double>(1.0 / zmq_publish_rate_hz_);
  if (last_zmq_publish_ != SteadyTime{} && now - last_zmq_publish_ < interval) {
    return true;
  }
  last_zmq_publish_ = now;
  std::ostringstream stream;
  stream << std::setprecision(17) << "{\"positions\":{";
  for (std::size_t arm_index = 0; arm_index < arm_indices_.size(); ++arm_index) {
    if (arm_index != 0) {
      stream << ',';
    }
    stream << '\"' << info_.joints[arm_indices_[arm_index]].name << "\":" << targets[arm_index];
  }
  stream << "}}";
  const std::string payload = stream.str();
  const int result = zmq_send(zmq_socket_, payload.data(), payload.size(), ZMQ_DONTWAIT);
  if (result >= 0 || errno == EAGAIN) {
    return true;
  }
  RCLCPP_ERROR_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000, "ZMQ send failed: %s", zmq_strerror(errno));
  return false;
}

void X2SystemHardware::enter_fault(const std::string & reason)
{
  if (fault_latched_.exchange(true)) {
    return;
  }
  active_ = false;
  RCLCPP_ERROR(node_->get_logger(), "X2 hardware fault: %s", reason.c_str());
  if (command_transport_ != "ros_topic" || fault_publish_duration_sec_ <= 0.0) {
    return;
  }
  const std::vector<double> zeros(arm_indices_.size(), 0.0);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(fault_publish_duration_sec_));
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    publish_ros_commands(zeros, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace agibot_x2_ros2_control

PLUGINLIB_EXPORT_CLASS(
  agibot_x2_ros2_control::X2SystemHardware, hardware_interface::SystemInterface)
