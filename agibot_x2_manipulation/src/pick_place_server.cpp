#include "agibot_x2_manipulation/box_geometry.hpp"

#include <agibot_x2_manipulation_msgs/action/pick.hpp>
#include <agibot_x2_manipulation_msgs/action/pick_place.hpp>
#include <agibot_x2_manipulation_msgs/action/place.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <agibot_x2_manipulation_msgs/srv/recover_manipulation_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

using PickPlace = agibot_x2_manipulation_msgs::action::PickPlace;
using Pick = agibot_x2_manipulation_msgs::action::Pick;
using Place = agibot_x2_manipulation_msgs::action::Place;
using ManipulationState = agibot_x2_manipulation_msgs::msg::ManipulationState;
using RecoverManipulationState =
  agibot_x2_manipulation_msgs::srv::RecoverManipulationState;
using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<Place>;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

constexpr uint16_t kSuccess = 0;
constexpr uint16_t kNoStableBoxPose = 2;
constexpr uint16_t kInvalidGoal = 3;
constexpr uint16_t kPlanningFailed = 4;
constexpr uint16_t kExecutionFailed = 5;
constexpr uint16_t kAttachmentFailed = 6;
constexpr uint16_t kSafetyAbort = 7;
constexpr uint16_t kInvalidState = 8;
constexpr uint16_t kRecoveryRequired = 9;

struct TaskOutcome
{
  bool success{false};
  uint16_t code{kSafetyAbort};
  std::string message;
  bool object_held{false};
  geometry_msgs::msg::PoseStamped achieved_pose;
};

using FeedbackFunction = std::function<void(
    const std::string &, float, const geometry_msgs::msg::PoseStamped &)>;
using CancelFunction = std::function<bool()>;

template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() < 1e-9) {
    throw std::invalid_argument("pose quaternion has zero length");
  }
  result.linear() = quaternion.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation = tf2::toMsg(Eigen::Quaterniond(pose.linear()).normalized());
  return result;
}

std::string defaultStateFile()
{
  const char * ros_home = std::getenv("ROS_HOME");
  if (ros_home && ros_home[0] != '\0') {
    return std::string(ros_home) + "/agibot_x2_manipulation_state";
  }
  const char * user_home = std::getenv("HOME");
  return user_home && user_home[0] != '\0' ?
         std::string(user_home) + "/.ros/agibot_x2_manipulation_state" :
         "/tmp/agibot_x2_manipulation_state";
}

}  // namespace

class PickPlaceServer
{
public:
  explicit PickPlaceServer(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_),
    move_group_(node, parameter<std::string>(node, "planning_group", "dual_arm"))
  {
    planning_frame_ = parameter<std::string>(node_, "planning_frame", "base_link");
    box_pose_topic_ = parameter<std::string>(node_, "box_pose_topic", "/box_pose");
    left_group_name_ = parameter<std::string>(node_, "left_group", "left_arm");
    right_group_name_ = parameter<std::string>(node_, "right_group", "right_arm");
    left_tcp_ = parameter<std::string>(node_, "left_tcp", "left_hand_tcp_link");
    right_tcp_ = parameter<std::string>(node_, "right_tcp", "right_hand_tcp_link");
    box_id_ = parameter<std::string>(node_, "box_id", "grasp_box");
    const auto dimensions = parameter<std::vector<double>>(
      node_,
      "box_dimensions", {0.30, 0.20, 0.15});
    if (dimensions.size() != 3U) {
      throw std::runtime_error("box_dimensions must contain [length, width, height]");
    }
    dimensions_ = {dimensions[0], dimensions[1], dimensions[2]};
    pregrasp_distance_ = parameter<double>(node_, "pregrasp_distance", 0.08);
    contact_height_offset_ = parameter<double>(node_, "contact_height_offset", 0.0);
    lift_height_ = parameter<double>(node_, "lift_height", 0.05);
    cartesian_step_ = parameter<double>(node_, "cartesian_step", 0.01);
    max_pose_age_ = parameter<double>(node_, "maximum_box_pose_age", 0.50);
    ik_timeout_ = parameter<double>(node_, "ik_timeout", 0.05);
    pregrasp_ik_attempts_ = parameter<int>(node_, "pregrasp_ik_attempts", 20);
    max_joint_step_ = parameter<double>(node_, "maximum_joint_step", 0.35);
    velocity_scaling_ = parameter<double>(node_, "velocity_scaling", 0.10);
    acceleration_scaling_ = parameter<double>(node_, "acceleration_scaling", 0.10);
    const auto carry_pose = parameter<std::vector<double>>(
      node_, "carry_box_pose", {0.35, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0});
    if (carry_pose.size() != 7U) {
      throw std::runtime_error("carry_box_pose must contain [x, y, z, qx, qy, qz, qw]");
    }
    geometry_msgs::msg::Pose carry_pose_message;
    carry_pose_message.position.x = carry_pose[0];
    carry_pose_message.position.y = carry_pose[1];
    carry_pose_message.position.z = carry_pose[2];
    carry_pose_message.orientation.x = carry_pose[3];
    carry_pose_message.orientation.y = carry_pose[4];
    carry_pose_message.orientation.z = carry_pose[5];
    carry_pose_message.orientation.w = carry_pose[6];
    carry_pose_ = toEigen(carry_pose_message);
    recovery_position_tolerance_ = parameter<double>(
      node_, "recovery_position_tolerance", 0.04);
    recovery_angular_tolerance_ = parameter<double>(
      node_, "recovery_angular_tolerance", 0.1745329252);
    state_file_ = parameter<std::string>(node_, "state_file", defaultStateFile());
    initial_state_ = parameter<std::string>(node_, "initial_state", "empty");
    post_place_named_target_ = parameter<std::string>(
      node_, "post_place_named_target", "zero");
    if (initial_state_ != "empty" && initial_state_ != "unknown") {
      throw std::runtime_error("initial_state must be 'empty' or 'unknown'");
    }
    simulate_attachment_ = parameter<bool>(node_, "simulate_ideal_attachment", true);
    const auto attach_service = parameter<std::string>(
      node_, "attach_service", "/mujoco_grasp/attach");
    const auto detach_service = parameter<std::string>(
      node_, "detach_service", "/mujoco_grasp/detach");

    move_group_.setPoseReferenceFrame(planning_frame_);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_.setPlanningTime(10.0);
    const auto model = move_group_.getRobotModel();
    if (!model->hasJointModelGroup(move_group_.getName()) ||
      !model->hasJointModelGroup(left_group_name_) ||
      !model->hasJointModelGroup(right_group_name_) ||
      !model->hasLinkModel(left_tcp_) || !model->hasLinkModel(right_tcp_))
    {
      throw std::runtime_error("configured arm groups or TCP links do not exist in the robot model");
    }
    const auto * dual_group = model->getJointModelGroup(move_group_.getName());
    if (!dual_group->isSubgroup(left_group_name_) || !dual_group->isSubgroup(right_group_name_))
    {
      throw std::runtime_error(
              "planning_group must contain the configured left and right arm groups");
    }
    if (pregrasp_ik_attempts_ < 1) {
      throw std::runtime_error("pregrasp_ik_attempts must be at least 1");
    }
    const auto named_targets = move_group_.getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(), post_place_named_target_) == named_targets.end())
    {
      throw std::runtime_error(
              "post_place_named_target is not defined for planning_group: " +
              post_place_named_target_);
    }

    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", "x2_pick_place_scene_monitor");
    if (!scene_monitor_->getPlanningScene()) {
      throw std::runtime_error("failed to create MoveIt planning scene monitor");
    }
    scene_monitor_->startStateMonitor();
    scene_monitor_->startSceneMonitor();
    scene_monitor_->startWorldGeometryMonitor();

    box_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      box_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_box_pose_ = *message;
        have_box_pose_ = true;
      });
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/grasp_markers", 10);
    state_pub_ = node_->create_publisher<ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local());
    attach_client_ = node_->create_client<std_srvs::srv::Trigger>(attach_service);
    detach_client_ = node_->create_client<std_srvs::srv::Trigger>(detach_service);

    initializeState();

    pick_action_server_ = rclcpp_action::create_server<Pick>(
      node_, "pick_box",
      std::bind(&PickPlaceServer::onPickGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onPickCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPickAccepted, this, std::placeholders::_1));
    place_action_server_ = rclcpp_action::create_server<Place>(
      node_, "place_box",
      std::bind(&PickPlaceServer::onPlaceGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onPlaceCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPlaceAccepted, this, std::placeholders::_1));
    pick_place_action_server_ = rclcpp_action::create_server<PickPlace>(
      node_, "pick_place",
      std::bind(&PickPlaceServer::onPickPlaceGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onPickPlaceCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPickPlaceAccepted, this, std::placeholders::_1));
    recovery_service_ = node_->create_service<RecoverManipulationState>(
      "/recover_manipulation_state",
      std::bind(
        &PickPlaceServer::recoverState, this, std::placeholders::_1, std::placeholders::_2));
    publishState();
  }

private:
  bool reserveGoal()
  {
    bool expected = false;
    return busy_.compare_exchange_strong(expected, true);
  }

  rclcpp_action::GoalResponse onPickGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal>)
  {
    return reserveGoal() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse onPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal>)
  {
    return reserveGoal() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse onPickPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal>)
  {
    return reserveGoal() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse cancelGoal(const std::shared_ptr<GoalHandleT> &)
  {
    move_group_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse onPickCancel(const std::shared_ptr<PickGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  rclcpp_action::CancelResponse onPlaceCancel(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  rclcpp_action::CancelResponse onPickPlaceCancel(
    const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  void onPickAccepted(const std::shared_ptr<PickGoalHandle> goal)
  {
    std::thread([this, goal]() {
      executePick(goal);
      busy_.store(false);
    }).detach();
  }

  void onPlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {
      executePlace(goal);
      busy_.store(false);
    }).detach();
  }

  void onPickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {
      executePickPlace(goal);
      busy_.store(false);
    }).detach();
  }

  void initializeState()
  {
    std::ifstream input(state_file_);
    std::string saved;
    input >> saved;
    if (saved == "EMPTY") {
      state_.store(ManipulationState::EMPTY);
      state_detail_ = "ready to pick";
    } else if (saved == "HOLDING") {
      state_.store(ManipulationState::UNKNOWN);
      state_detail_ = "previous session may have held an object; recovery confirmation required";
    } else if (initial_state_ == "empty") {
      state_.store(ManipulationState::EMPTY);
      state_detail_ = "initial state configured empty";
    } else {
      state_.store(ManipulationState::UNKNOWN);
      state_detail_ = "initial state requires operator confirmation";
    }
  }

  void persistState(uint8_t state)
  {
    try {
      const std::filesystem::path path(state_file_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      const auto temporary = path.string() + ".tmp";
      {
        std::ofstream output(temporary, std::ios::trunc);
        if (!output) {
          throw std::runtime_error("cannot open state file");
        }
        output << (state == ManipulationState::EMPTY ? "EMPTY\n" : "HOLDING\n");
      }
      std::filesystem::rename(temporary, path);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to persist manipulation state: %s", exception.what());
    }
  }

  void publishState()
  {
    ManipulationState message;
    message.state = state_.load();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      message.detail = state_detail_;
    }
    state_pub_->publish(message);
  }

  void setState(uint8_t state, const std::string & detail, bool persist = true)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_detail_ = detail;
    }
    state_.store(state);
    if (persist && state != ManipulationState::UNKNOWN) {
      persistState(state);
    }
    publishState();
  }

  geometry_msgs::msg::PoseStamped stampedPose(const Eigen::Isometry3d & pose) const
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = planning_frame_;
    result.header.stamp = node_->now();
    result.pose = toPoseMsg(pose);
    return result;
  }

  bool recoverHolding(std::string & error)
  {
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable";
      return false;
    }
    current->update();
    const auto grasp = computeGraspGeometry(
      carry_pose_, dimensions_, 0.0, contact_height_offset_);
    const auto within_tolerance = [this](
      const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected) {
        const double position_error = (actual.translation() - expected.translation()).norm();
        const Eigen::Quaterniond qa(actual.linear());
        const Eigen::Quaterniond qb(expected.linear());
        const double angle_error = 2.0 * std::acos(
          std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
        return position_error <= recovery_position_tolerance_ &&
               angle_error <= recovery_angular_tolerance_;
      };
    if (!within_tolerance(current->getGlobalLinkTransform(left_tcp_), grasp.left_contact) ||
      !within_tolerance(current->getGlobalLinkTransform(right_tcp_), grasp.right_contact))
    {
      error = "TCP poses do not match the configured carry pose within recovery tolerances";
      return false;
    }
    applyBox(carry_pose_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!move_group_.attachObject(
        box_id_, left_tcp_, {"left_hand_pad_link", left_tcp_, "right_hand_pad_link", right_tcp_}))
    {
      error = "failed to reconstruct the MoveIt attached object";
      return false;
    }
    held_pose_ = stampedPose(carry_pose_);
    return true;
  }

  void recoverState(
    const std::shared_ptr<RecoverManipulationState::Request> request,
    std::shared_ptr<RecoverManipulationState::Response> response)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->message = "manipulation server is busy";
      return;
    }
    if (request->requested_state == RecoverManipulationState::Request::CONFIRM_EMPTY) {
      move_group_.detachObject(box_id_);
      planning_scene_interface_.removeCollisionObjects({box_id_});
      held_pose_ = geometry_msgs::msg::PoseStamped();
      setState(ManipulationState::EMPTY, "operator confirmed that no object is held");
      response->success = true;
      response->message = "manipulation state recovered as EMPTY";
    } else if (request->requested_state == RecoverManipulationState::Request::CONFIRM_HOLDING) {
      std::string error;
      if (recoverHolding(error)) {
        setState(ManipulationState::HOLDING, "operator confirmed object at carry pose");
        response->success = true;
        response->message = "manipulation state recovered as HOLDING";
      } else {
        response->message = error;
      }
    } else {
      response->message = "requested_state must be CONFIRM_EMPTY or CONFIRM_HOLDING";
    }
    busy_.store(false);
  }

  bool stableBoxPose(geometry_msgs::msg::PoseStamped & pose)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!have_box_pose_ || (node_->now() - latest_box_pose_.header.stamp).seconds() > max_pose_age_) {
      return false;
    }
    pose.header = latest_box_pose_.header;
    pose.pose = latest_box_pose_.pose.pose;
    return pose.header.frame_id == planning_frame_;
  }

  bool transformGoalPose(
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output,
    std::string & error)
  {
    if (input.header.frame_id.empty()) {
      error = "place_pose.frame_id is empty";
      return false;
    }
    try {
      if (input.header.frame_id == planning_frame_) {
        output = input;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          planning_frame_, input.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(input, output, transform);
      }
      output.header.frame_id = planning_frame_;
      return true;
    } catch (const tf2::TransformException & exception) {
      error = exception.what();
      return false;
    }
  }

  void applyBox(const Eigen::Isometry3d & pose)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = planning_frame_;
    object.id = box_id_;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {dimensions_.length, dimensions_.width, dimensions_.height};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(toPoseMsg(pose));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_interface_.applyCollisionObject(object);
  }

  bool callAttachmentService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, std::string & error)
  {
    if (!simulate_attachment_) {
      return true;
    }
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      error = "MuJoCo attachment service is unavailable";
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      error = "MuJoCo attachment service timed out";
      return false;
    }
    const auto response = future.get();
    error = response->message;
    return response->success;
  }

  bool collisionFree(moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    if (ignore_box) {
      acm.setEntry(box_id_, true);
    } else if (allow_pad_contact) {
      acm.setEntry(box_id_, std::vector<std::string>{
        "left_hand_pad_link", left_tcp_, "right_hand_pad_link", right_tcp_}, true);
    }
    collision_detection::CollisionRequest request;
    request.group_name = move_group_.getName();
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state, acm);
    return !result.collision;
  }

  bool setFromDualArmIK(
    moveit::core::RobotState & state, const Eigen::Isometry3d & left_pose,
    const Eigen::Isometry3d & right_pose, bool require_continuity = false) const
  {
    const auto * left_group = state.getJointModelGroup(left_group_name_);
    const auto * right_group = state.getJointModelGroup(right_group_name_);
    const std::vector<double> left_consistency(
      left_group->getVariableCount(), max_joint_step_);
    const std::vector<double> right_consistency(
      right_group->getVariableCount(), max_joint_step_);
    state.update();
    const bool left_solved = require_continuity ?
      state.setFromIK(left_group, left_pose, left_tcp_, left_consistency, ik_timeout_) :
      state.setFromIK(left_group, left_pose, left_tcp_, ik_timeout_);
    if (!left_solved) {
      return false;
    }
    state.update();
    const bool right_solved = require_continuity ?
      state.setFromIK(right_group, right_pose, right_tcp_, right_consistency, ik_timeout_) :
      state.setFromIK(right_group, right_pose, right_tcp_, ik_timeout_);
    if (!right_solved) {
      return false;
    }
    state.update();
    return true;
  }

  bool solvePregraspIK(
    const moveit::core::RobotState & seed, const GraspGeometry & grasp,
    moveit::core::RobotState & solution, std::string & error)
  {
    const auto * dual_group = seed.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounded = false;
    bool found_collision_free = false;
    double best_distance = std::numeric_limits<double>::infinity();
    moveit::core::RobotState best(seed);

    for (int attempt = 0; attempt < pregrasp_ik_attempts_; ++attempt) {
      moveit::core::RobotState candidate(seed);
      if (attempt > 0) {
        candidate.setToRandomPositions(dual_group);
      }
      if (!setFromDualArmIK(candidate, grasp.left_pregrasp, grasp.right_pregrasp)) {
        continue;
      }
      found_ik = true;
      if (!candidate.satisfiesBounds(dual_group)) {
        continue;
      }
      found_bounded = true;
      if (!collisionFree(candidate, false, false)) {
        continue;
      }
      found_collision_free = true;
      const double distance = candidate.distance(seed, dual_group);
      if (distance < best_distance) {
        best_distance = distance;
        best = candidate;
      }
    }

    if (found_collision_free) {
      solution = best;
      solution.update();
      return true;
    }
    if (!found_ik) {
      error = "no dual-arm pregrasp IK solution";
    } else if (!found_bounded) {
      error = "all dual-arm pregrasp IK solutions violate joint bounds";
    } else {
      error = "all bounded dual-arm pregrasp IK solutions are in collision";
    }
    return false;
  }

  bool solveGraspWaypoint(
    moveit::core::RobotState & state, const GraspGeometry & grasp,
    const moveit::core::RobotState & previous, bool allow_pad_contact, bool ignore_box)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    if (!setFromDualArmIK(state, grasp.left_contact, grasp.right_contact, true)) {
      RCLCPP_WARN(node_->get_logger(), "Cartesian waypoint rejected: dual-arm IK failed");
      return false;
    }
    double largest_joint_step = 0.0;
    for (const auto * joint : dual_group->getActiveJointModels()) {
      largest_joint_step = std::max(
        largest_joint_step,
        joint->distance(previous.getJointPositions(joint), state.getJointPositions(joint)));
    }
    if (largest_joint_step > max_joint_step_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Cartesian waypoint rejected: largest joint step %.3f exceeds maximum %.3f",
        largest_joint_step, max_joint_step_);
      return false;
    }
    if (!collisionFree(state, allow_pad_contact, ignore_box)) {
      RCLCPP_WARN(node_->get_logger(), "Cartesian waypoint rejected: state is in collision");
      return false;
    }
    return true;
  }

  bool appendObjectSegment(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, bool allow_pad_contact,
    bool ignore_box)
  {
    const double distance = (to.translation() - from.translation()).norm();
    const Eigen::Quaterniond qa(from.linear());
    const Eigen::Quaterniond qb(to.linear());
    const double angle = 2.0 * std::acos(std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
    const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
      distance / cartesian_step_, angle / 0.0872664626))));
    for (int index = 1; index <= steps; ++index) {
      const auto object_pose = interpolatePose(from, to, static_cast<double>(index) / steps);
      const auto grasp = computeGraspGeometry(
        object_pose, dimensions_, 0.0, contact_height_offset_);
      const moveit::core::RobotState previous(state);
      if (!solveGraspWaypoint(state, grasp, previous, allow_pad_contact, ignore_box)) {
        return false;
      }
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    return true;
  }

  bool buildApproach(
    const moveit::core::RobotState & start, const GraspGeometry & target,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    const int steps = std::max(1, static_cast<int>(std::ceil(pregrasp_distance_ / cartesian_step_)));
    for (int index = 1; index <= steps; ++index) {
      const double t = static_cast<double>(index) / steps;
      GraspGeometry waypoint = target;
      waypoint.left_contact = interpolatePose(target.left_pregrasp, target.left_contact, t);
      waypoint.right_contact = interpolatePose(target.right_pregrasp, target.right_contact, t);
      const moveit::core::RobotState previous(state);
      if (!solveGraspWaypoint(state, waypoint, previous, true, false)) {
        return false;
      }
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & place_pose, bool plan_only,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += lift_height_;
    Eigen::Isometry3d place_lift = place_pose;
    place_lift.translation().z() += lift_height_;
    const bool ignore_box = plan_only;
    if (!appendObjectSegment(trajectory, state, pick_pose, pick_lift, false, ignore_box) ||
      !appendObjectSegment(trajectory, state, pick_lift, place_lift, false, ignore_box) ||
      !appendObjectSegment(trajectory, state, place_lift, place_pose, false, ignore_box))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildCarryTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & carry_pose, bool plan_only,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += lift_height_;
    if (!appendObjectSegment(trajectory, state, pick_pose, pick_lift, false, plan_only) ||
      !appendObjectSegment(trajectory, state, pick_lift, carry_pose, false, plan_only))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildPlaceTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & held_pose,
    const Eigen::Isometry3d & place_pose, moveit_msgs::msg::RobotTrajectory & output,
    moveit::core::RobotState & end_state)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d place_lift = place_pose;
    place_lift.translation().z() += lift_height_;
    if (!appendObjectSegment(trajectory, state, held_pose, place_lift, false, false) ||
      !appendObjectSegment(trajectory, state, place_lift, place_pose, false, false))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  moveit::core::RobotState stateAtEnd(
    const moveit::core::RobotState & start, const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    moveit::core::RobotState result(start);
    if (!trajectory.joint_trajectory.points.empty()) {
      result.setVariablePositions(
        trajectory.joint_trajectory.joint_names,
        trajectory.joint_trajectory.points.back().positions);
      result.update();
    }
    return result;
  }

  void publishGraspMarkers(const geometry_msgs::msg::PoseStamped & box, const GraspGeometry & grasp)
  {
    visualization_msgs::msg::MarkerArray array;
    const std::array<std::pair<Eigen::Isometry3d, std::array<float, 3>>, 4> poses{{
      {grasp.left_contact, {0.0F, 1.0F, 0.0F}},
      {grasp.right_contact, {0.0F, 1.0F, 0.0F}},
      {grasp.left_pregrasp, {1.0F, 0.7F, 0.0F}},
      {grasp.right_pregrasp, {1.0F, 0.7F, 0.0F}}}};
    int id = 0;
    for (const auto & item : poses) {
      const bool left_hand = (id == 0 || id == 2);
      visualization_msgs::msg::Marker marker;
      marker.header = box.header;
      marker.ns = "box_grasps";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = toPoseMsg(item.first);
      const Eigen::Vector3d tcp_axis(0.0, left_hand ? -1.0 : 1.0, 0.0);
      const Eigen::Vector3d contact_axis = item.first.linear() * tcp_axis;
      marker.pose.orientation = tf2::toMsg(
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), contact_axis));
      marker.scale.x = 0.06;
      marker.scale.y = 0.012;
      marker.scale.z = 0.012;
      marker.color.r = item.second[0];
      marker.color.g = item.second[1];
      marker.color.b = item.second[2];
      marker.color.a = 1.0F;
      array.markers.push_back(marker);
    }
    marker_pub_->publish(array);
  }

  TaskOutcome outcome(
    bool success, uint16_t code, const std::string & message,
    const geometry_msgs::msg::PoseStamped & pose = geometry_msgs::msg::PoseStamped()) const
  {
    TaskOutcome result;
    result.success = success;
    result.code = code;
    result.message = message;
    result.object_held = state_.load() == ManipulationState::HOLDING ||
      state_.load() == ManipulationState::RECOVERY_REQUIRED;
    result.achieved_pose = pose;
    if (result.achieved_pose.header.frame_id.empty()) {
      result.achieved_pose.header.frame_id = planning_frame_;
    }
    return result;
  }

  bool planPickPath(
    const geometry_msgs::msg::PoseStamped & box_message, const Eigen::Isometry3d & pick_pose,
    moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
    moveit_msgs::msg::RobotTrajectory & approach, moveit::core::RobotState & contact_end,
    std::string & error)
  {
    const auto pick_grasp = computeGraspGeometry(
      pick_pose, dimensions_, pregrasp_distance_, contact_height_offset_);
    publishGraspMarkers(box_message, pick_grasp);
    applyBox(pick_pose);
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable";
      return false;
    }
    current->update();
    moveit::core::RobotState pregrasp_goal(*current);
    if (!solvePregraspIK(*current, pick_grasp, pregrasp_goal, error)) {
      return false;
    }
    move_group_.setStartState(*current);
    move_group_.clearPoseTargets();
    if (!move_group_.setJointValueTarget(pregrasp_goal)) {
      error = "pregrasp joint target violates MoveIt bounds";
      return false;
    }
    if (move_group_.plan(pregrasp_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "pregrasp joint-space planning failed";
      return false;
    }
    auto pregrasp_end = stateAtEnd(*current, pregrasp_plan.trajectory_);
    contact_end = pregrasp_end;
    if (!buildApproach(pregrasp_end, pick_grasp, approach, contact_end)) {
      error = "dual-arm Cartesian approach failed";
      return false;
    }
    return true;
  }

  void updateHeldPoseFromRobot()
  {
    auto current = move_group_.getCurrentState(1.0);
    if (!current) {
      return;
    }
    current->update();
    const auto carry_grasp = computeGraspGeometry(
      carry_pose_, dimensions_, 0.0, contact_height_offset_);
    const Eigen::Isometry3d box_to_left_tcp = carry_pose_.inverse() * carry_grasp.left_contact;
    const Eigen::Isometry3d estimated =
      current->getGlobalLinkTransform(left_tcp_) * box_to_left_tcp.inverse();
    held_pose_ = stampedPose(estimated);
  }

  TaskOutcome runPick(
    bool plan_only, const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "Pick requires manipulation state EMPTY");
    }
    geometry_msgs::msg::PoseStamped box_message;
    if (!stableBoxPose(box_message)) {
      return outcome(false, kNoStableBoxPose, "no fresh stable box pose");
    }
    Eigen::Isometry3d pick_pose;
    try {
      pick_pose = toEigen(box_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }

    feedback("planning_pregrasp", 0.15F, box_message);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable");
    }
    moveit::core::RobotState contact_end(*current);
    std::string error;
    if (!planPickPath(box_message, pick_pose, pregrasp_plan, approach, contact_end, error)) {
      return outcome(false, kPlanningFailed, error);
    }
    moveit_msgs::msg::RobotTrajectory carry;
    moveit::core::RobotState carry_end(contact_end);
    if (!buildCarryTransport(contact_end, pick_pose, carry_pose_, true, carry, carry_end)) {
      return outcome(false, kPlanningFailed, "closed-chain carry planning failed");
    }
    if (plan_only) {
      return outcome(true, kSuccess, "pick path to carry pose is feasible", stampedPose(carry_pose_));
    }

    feedback("executing_pregrasp", 0.30F, box_message);
    if (move_group_.execute(pregrasp_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return outcome(false, kExecutionFailed, "pregrasp execution failed");
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before approach");
    }
    const auto pick_grasp = computeGraspGeometry(
      pick_pose, dimensions_, pregrasp_distance_, contact_height_offset_);
    current = move_group_.getCurrentState(2.0);
    if (!current || !buildApproach(*current, pick_grasp, approach, contact_end)) {
      return outcome(false, kSafetyAbort, "approach revalidation failed");
    }
    feedback("approaching", 0.45F, box_message);
    if (move_group_.execute(approach) != moveit::core::MoveItErrorCode::SUCCESS) {
      return outcome(false, kExecutionFailed, "approach execution failed");
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before attachment");
    }

    feedback("attaching", 0.60F, box_message);
    if (!callAttachmentService(attach_client_, error)) {
      return outcome(false, kAttachmentFailed, error);
    }
    held_pose_ = box_message;
    if (!move_group_.attachObject(
        box_id_, left_tcp_, {"left_hand_pad_link", left_tcp_, "right_hand_pad_link", right_tcp_}))
    {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "physical attachment may exist but MoveIt attachment failed");
      return outcome(false, kRecoveryRequired, "MoveIt object attachment failed; object was not released");
    }
    setState(ManipulationState::HOLDING, "box attached at pick pose");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after attachment");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }

    current = move_group_.getCurrentState(2.0);
    if (!current ||
      !buildCarryTransport(*current, pick_pose, carry_pose_, false, carry, carry_end))
    {
      setState(ManipulationState::RECOVERY_REQUIRED, "carry revalidation failed");
      updateHeldPoseFromRobot();
      return outcome(false, kRecoveryRequired, "carry revalidation failed; object remains held", held_pose_);
    }
    feedback("moving_to_carry", 0.80F, box_message);
    if (move_group_.execute(carry) != moveit::core::MoveItErrorCode::SUCCESS) {
      updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "carry execution failed");
      return outcome(false, kRecoveryRequired, "carry execution failed; object remains held", held_pose_);
    }
    held_pose_ = stampedPose(carry_pose_);
    setState(ManipulationState::HOLDING, "box held at carry pose");
    feedback("holding", 1.0F, held_pose_);
    return outcome(true, kSuccess, "box picked and moved to carry pose", held_pose_);
  }

  TaskOutcome runPlace(
    const geometry_msgs::msg::PoseStamped & requested_pose, bool plan_only,
    const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    const uint8_t current_state = state_.load();
    if (current_state != ManipulationState::HOLDING &&
      current_state != ManipulationState::RECOVERY_REQUIRED)
    {
      return outcome(false, kInvalidState, "Place requires a held object");
    }
    geometry_msgs::msg::PoseStamped place_message;
    std::string error;
    if (!transformGoalPose(requested_pose, place_message, error)) {
      return outcome(false, kInvalidGoal, error);
    }
    Eigen::Isometry3d place_pose;
    try {
      place_pose = toEigen(place_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }
    updateHeldPoseFromRobot();
    Eigen::Isometry3d from_pose;
    try {
      from_pose = toEigen(held_pose_.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kRecoveryRequired, exception.what(), held_pose_);
    }
    feedback("planning_place", 0.15F, held_pose_);
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable", held_pose_);
    }
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(*current);
    if (!buildPlaceTransport(*current, from_pose, place_pose, transport, place_end)) {
      return outcome(false, kPlanningFailed, "closed-chain place planning failed", held_pose_);
    }
    if (plan_only) {
      return outcome(true, kSuccess, "place path is feasible", place_message);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "place canceled before motion", held_pose_);
    }
    feedback("moving_to_place", 0.45F, held_pose_);
    if (move_group_.execute(transport) != moveit::core::MoveItErrorCode::SUCCESS) {
      updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "place motion failed");
      return outcome(false, kRecoveryRequired, "place motion failed; object remains held", held_pose_);
    }
    held_pose_ = place_message;
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "place canceled before detachment");
      return outcome(false, kRecoveryRequired, "place canceled; object remains held", held_pose_);
    }

    feedback("detaching", 0.70F, place_message);
    if (!callAttachmentService(detach_client_, error)) {
      setState(ManipulationState::RECOVERY_REQUIRED, "detach service failed");
      return outcome(false, kRecoveryRequired, error + "; object remains held", held_pose_);
    }
    move_group_.detachObject(box_id_);
    applyBox(place_pose);
    setState(ManipulationState::EMPTY, "box placed; retreat in progress");

    const auto place_grasp = computeGraspGeometry(
      place_pose, dimensions_, pregrasp_distance_, contact_height_offset_);
    current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "box placed, but current state is unavailable", place_message);
    }
    GraspGeometry retreat_target = place_grasp;
    std::swap(retreat_target.left_contact, retreat_target.left_pregrasp);
    std::swap(retreat_target.right_contact, retreat_target.right_pregrasp);
    moveit_msgs::msg::RobotTrajectory retreat;
    moveit::core::RobotState retreat_end(*current);
    feedback("retreating", 0.80F, place_message);
    if (!buildApproach(*current, retreat_target, retreat, retreat_end) ||
      move_group_.execute(retreat) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      return outcome(false, kExecutionFailed, "box placed, but retreat failed", place_message);
    }
    held_pose_ = geometry_msgs::msg::PoseStamped();
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; return to zero canceled");
      return outcome(
        false, kExecutionFailed, "box placed, but return to zero was canceled", place_message);
    }

    feedback("returning_to_zero", 0.92F, place_message);
    current = move_group_.getCurrentState(2.0);
    if (!current) {
      setState(ManipulationState::EMPTY, "box placed; current state unavailable for zero motion");
      return outcome(
        false, kSafetyAbort, "box placed, but current state is unavailable before zero motion",
        place_message);
    }
    move_group_.setStartState(*current);
    move_group_.clearPoseTargets();
    if (!move_group_.setNamedTarget(post_place_named_target_)) {
      setState(ManipulationState::EMPTY, "box placed; zero target is unavailable");
      return outcome(
        false, kPlanningFailed, "box placed, but the zero target is unavailable", place_message);
    }
    moveit::planning_interface::MoveGroupInterface::Plan zero_plan;
    if (move_group_.plan(zero_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setState(ManipulationState::EMPTY, "box placed; return-to-zero planning failed");
      return outcome(
        false, kPlanningFailed, "box placed, but return-to-zero planning failed", place_message);
    }
    if (move_group_.execute(zero_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setState(ManipulationState::EMPTY, "box placed; return-to-zero execution failed");
      return outcome(
        false, kExecutionFailed, "box placed, but return-to-zero execution failed", place_message);
    }
    setState(ManipulationState::EMPTY, "ready to pick; arms at zero");
    feedback("complete", 1.0F, place_message);
    return outcome(true, kSuccess, "box placed and arms returned to zero", place_message);
  }

  TaskOutcome planCompletePath(const geometry_msgs::msg::PoseStamped & requested_place)
  {
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "PickPlace requires manipulation state EMPTY");
    }
    geometry_msgs::msg::PoseStamped box_message;
    if (!stableBoxPose(box_message)) {
      return outcome(false, kNoStableBoxPose, "no fresh stable box pose");
    }
    geometry_msgs::msg::PoseStamped place_message;
    std::string error;
    if (!transformGoalPose(requested_place, place_message, error)) {
      return outcome(false, kInvalidGoal, error);
    }
    Eigen::Isometry3d pick_pose;
    Eigen::Isometry3d place_pose;
    try {
      pick_pose = toEigen(box_message.pose);
      place_pose = toEigen(place_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable");
    }
    moveit::core::RobotState contact_end(*current);
    if (!planPickPath(box_message, pick_pose, pregrasp_plan, approach, contact_end, error)) {
      return outcome(false, kPlanningFailed, error);
    }
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(contact_end);
    if (!buildTransport(contact_end, pick_pose, place_pose, true, transport, place_end)) {
      return outcome(false, kPlanningFailed, "closed-chain transport planning failed");
    }
    return outcome(true, kSuccess, "complete pick/place path is feasible", place_message);
  }

  template<typename GoalHandleT, typename ResultT>
  void finishGoal(
    const std::shared_ptr<GoalHandleT> & goal, const TaskOutcome & task,
    const std::shared_ptr<ResultT> & result)
  {
    result->success = task.success;
    result->error_code = task.code;
    result->message = task.message;
    result->achieved_pose = task.achieved_pose;
    if constexpr (std::is_same_v<ResultT, Pick::Result> || std::is_same_v<ResultT, Place::Result>) {
      result->object_held = task.object_held;
    }
    if (goal->is_canceling()) {
      goal->canceled(result);
    } else if (task.success) {
      goal->succeed(result);
    } else {
      goal->abort(result);
    }
  }

  void executePick(const std::shared_ptr<PickGoalHandle> & goal)
  {
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Pick::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    const auto task = runPick(
      goal->get_goal()->plan_only, feedback, [goal]() {return goal->is_canceling();});
    finishGoal(goal, task, std::make_shared<Pick::Result>());
  }

  void executePlace(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Place::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    const auto task = runPlace(
      goal->get_goal()->place_pose, goal->get_goal()->plan_only, feedback,
      [goal]() {return goal->is_canceling();});
    finishGoal(goal, task, std::make_shared<Place::Result>());
  }

  void executePickPlace(const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    TaskOutcome task;
    if (goal->get_goal()->plan_only) {
      task = planCompletePath(goal->get_goal()->place_pose);
    } else {
      const FeedbackFunction pick_feedback = [goal](
        const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
          auto message = std::make_shared<PickPlace::Feedback>();
          message->stage = "pick/" + stage;
          message->progress = progress * 0.5F;
          message->box_pose = pose;
          goal->publish_feedback(message);
        };
      task = runPick(false, pick_feedback, [goal]() {return goal->is_canceling();});
      if (task.success) {
        const FeedbackFunction place_feedback = [goal](
          const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
            auto message = std::make_shared<PickPlace::Feedback>();
            message->stage = "place/" + stage;
            message->progress = 0.5F + progress * 0.5F;
            message->box_pose = pose;
            goal->publish_feedback(message);
          };
        task = runPlace(
          goal->get_goal()->place_pose, false, place_feedback,
          [goal]() {return goal->is_canceling();});
      }
      if (!task.success && task.object_held) {
        task.message += "; object remains held";
      }
    }
    finishGoal(goal, task, std::make_shared<PickPlace::Result>());
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string box_pose_topic_;
  std::string left_group_name_;
  std::string right_group_name_;
  std::string left_tcp_;
  std::string right_tcp_;
  std::string box_id_;
  BoxDimensions dimensions_;
  double pregrasp_distance_;
  double contact_height_offset_;
  double lift_height_;
  double cartesian_step_;
  double max_pose_age_;
  double ik_timeout_;
  int pregrasp_ik_attempts_;
  double max_joint_step_;
  double velocity_scaling_;
  double acceleration_scaling_;
  Eigen::Isometry3d carry_pose_{Eigen::Isometry3d::Identity()};
  double recovery_position_tolerance_;
  double recovery_angular_tolerance_;
  std::string state_file_;
  std::string initial_state_;
  std::string post_place_named_target_;
  bool simulate_attachment_;
  std::atomic<bool> busy_{false};
  std::atomic<uint8_t> state_{ManipulationState::UNKNOWN};
  std::mutex pose_mutex_;
  std::mutex state_mutex_;
  std::string state_detail_;
  bool have_box_pose_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped latest_box_pose_;
  geometry_msgs::msg::PoseStamped held_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr box_pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<ManipulationState>::SharedPtr state_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr attach_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detach_client_;
  rclcpp_action::Server<Pick>::SharedPtr pick_action_server_;
  rclcpp_action::Server<Place>::SharedPtr place_action_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_action_server_;
  rclcpp::Service<RecoverManipulationState>::SharedPtr recovery_service_;
};

}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<agibot_x2_manipulation::PickPlaceServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  server.reset();
  rclcpp::shutdown();
  return 0;
}
