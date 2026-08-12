#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/closed_chain_path_planner.hpp"
#include "agibot_x2_manipulation/execution_feedback.hpp"
#include "agibot_x2_manipulation/perception_readiness.hpp"
#include "agibot_x2_manipulation/planning_budget.hpp"
#include "agibot_x2_manipulation/reset_coordinator.hpp"
#include "agibot_x2_manipulation/reset_utils.hpp"

#include <agibot_x2_manipulation_msgs/action/pick.hpp>
#include <agibot_x2_manipulation_msgs/action/pick_place.hpp>
#include <agibot_x2_manipulation_msgs/action/place.hpp>
#include <agibot_x2_manipulation_msgs/action/reset_manipulation.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <agibot_x2_manipulation_msgs/srv/recover_manipulation_state.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/empty.hpp>
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
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
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
using ResetManipulation = agibot_x2_manipulation_msgs::action::ResetManipulation;
using ManipulationState = agibot_x2_manipulation_msgs::msg::ManipulationState;
using RecoverManipulationState =
  agibot_x2_manipulation_msgs::srv::RecoverManipulationState;
using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<Place>;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using ResetGoalHandle = rclcpp_action::ServerGoalHandle<ResetManipulation>;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using ExecuteTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

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

struct PlannedGrasp
{
  GraspCandidate candidate;
  double joint_limit_margin{0.0};
  double joint_distance{0.0};
};

// Endpoint acceptance is based on direct HAL measurements, not the
// joint-state broadcaster. The latter may republish cached ros2_control state
// after a physical feedback stream has stopped.
struct HalArmFeedbackSnapshot
{
  std::map<std::string, JointFeedback> joints;
  uint32_t measurement_sequence{0};
  bool valid{false};
};

using CarryRoute = ClosedChainRoute;

const char * carryRouteName(CarryRoute route)
{
  return closedChainRouteName(route);
}

struct AdaptiveCarryPlan
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  CarryRoute route{CarryRoute::DIRECT};
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::shared_ptr<moveit::core::RobotState> end_state;
};

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> callback)
  : callback_(std::move(callback)) {}

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

  ~ScopeExit()
  {
    run();
  }

  void run()
  {
    if (callback_) {
      auto callback = std::move(callback_);
      callback();
    }
  }

  void dismiss()
  {
    callback_ = nullptr;
  }

private:
  std::function<void()> callback_;
};

using FeedbackFunction = std::function<void(
    const std::string &, float, const geometry_msgs::msg::PoseStamped &)>;
using CancelFunction = std::function<bool()>;
using ContinuationFunction = std::function<bool(
    const moveit::core::RobotState &, const PlannedGrasp &, std::string &)>;

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

bool readTransform(std::istream & input, Eigen::Isometry3d & transform)
{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  input >> translation.x() >> translation.y() >> translation.z() >>
    rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();
  if (!input || !translation.allFinite() || !rotation.coeffs().allFinite() ||
    rotation.norm() < 1e-9)
  {
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  transform.linear() = rotation.normalized().toRotationMatrix();
  return true;
}

void writeTransform(std::ostream & output, const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond rotation(transform.linear());
  output << std::setprecision(17) << transform.translation().x() << ' '
         << transform.translation().y() << ' ' << transform.translation().z() << ' '
         << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z() << ' '
         << rotation.w() << '\n';
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
    grasp_position_tolerance_ = parameter<double>(node_, "grasp_position_tolerance", 0.015);
    grasp_orientation_tolerance_ = parameter<double>(
      node_, "grasp_orientation_tolerance", 0.0872664626);
    pregrasp_distance_tolerance_ = parameter<double>(
      node_, "pregrasp_distance_tolerance", 0.015);
    alternate_face_alignment_tolerance_ = parameter<double>(
      node_, "alternate_face_alignment_tolerance", 0.261799388);
    maximum_grasp_candidates_ = parameter<int>(node_, "maximum_grasp_candidates", 64);
    grasp_search_timeout_ = parameter<double>(node_, "grasp_search_timeout", 2.0);
    ik_attempts_per_candidate_ = parameter<int>(node_, "ik_attempts_per_candidate", 4);
    maximum_planning_candidates_ = parameter<int>(node_, "maximum_planning_candidates", 5);
    planning_time_per_candidate_ = parameter<double>(node_, "planning_time_per_candidate", 2.0);
    pregrasp_planning_timeout_ = parameter<double>(node_, "pregrasp_planning_timeout", 30.0);
    maximum_retry_candidates_ = parameter<int>(node_, "maximum_retry_candidates", 3);
    minimum_grasp_joint_margin_ = parameter<double>(
      node_, "minimum_grasp_joint_margin", 0.02);
    closed_chain_position_tolerance_ = parameter<double>(
      node_, "closed_chain_position_tolerance", 0.010);
    closed_chain_orientation_tolerance_ = parameter<double>(
      node_, "closed_chain_orientation_tolerance", 0.0523598776);
    closed_chain_position_step_ = parameter<double>(node_, "closed_chain_position_step", 0.005);
    closed_chain_orientation_step_ = parameter<double>(
      node_, "closed_chain_orientation_step", 0.0261799388);
    closed_chain_ik_attempts_ = parameter<int>(node_, "closed_chain_ik_attempts", 4);
    closed_chain_ik_timeout_ = parameter<double>(node_, "closed_chain_ik_timeout", 0.10);
    closed_chain_beam_width_ = parameter<int>(node_, "closed_chain_beam_width", 8);
    closed_chain_solutions_per_branch_ = parameter<int>(
      node_, "closed_chain_solutions_per_branch", 2);
    closed_chain_projection_limit_ = parameter<int>(node_, "closed_chain_projection_limit", 32);
    closed_chain_validation_position_step_ = parameter<double>(
      node_, "closed_chain_validation_position_step", 0.005);
    closed_chain_validation_orientation_step_ = parameter<double>(
      node_, "closed_chain_validation_orientation_step", 0.0174533);
    closed_chain_contact_position_error_ = parameter<double>(
      node_, "closed_chain_contact_position_error", 0.002);
    closed_chain_contact_orientation_error_ = parameter<double>(
      node_, "closed_chain_contact_orientation_error", 0.0174533);
    carry_search_timeout_ = parameter<double>(node_, "carry_search_timeout", 8.0);
    maximum_carry_candidates_ = parameter<int>(node_, "maximum_carry_candidates", 96);
    carry_search_x_range_ = parameter<double>(node_, "carry_search_x_range", 0.05);
    carry_search_y_range_ = parameter<double>(node_, "carry_search_y_range", 0.03);
    carry_search_z_lower_ = parameter<double>(node_, "carry_search_z_lower", 0.12);
    carry_search_z_upper_ = parameter<double>(node_, "carry_search_z_upper", 0.03);
    carry_search_orientation_tolerance_ = parameter<double>(
      node_, "carry_search_orientation_tolerance", 0.1745329252);
    const auto place_tolerance = parameter<std::vector<double>>(
      node_, "adaptive_place_position_tolerance", {0.015, 0.015, 0.005});
    if (place_tolerance.size() != 3U) {
      throw std::runtime_error("adaptive_place_position_tolerance must contain [x, y, z]");
    }
    adaptive_place_position_tolerance_ = Eigen::Vector3d(
      place_tolerance[0], place_tolerance[1], place_tolerance[2]);
    adaptive_place_yaw_tolerance_ = parameter<double>(
      node_, "adaptive_place_yaw_tolerance", 0.0872664626);
    max_joint_step_ = parameter<double>(node_, "maximum_joint_step", 0.35);
    allow_execution_ = parameter<bool>(node_, "allow_execution", false);
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
    reset_named_target_ = parameter<std::string>(node_, "reset_named_target", "zero");
    reset_preemption_timeout_ = parameter<double>(node_, "reset_preemption_timeout", 15.0);
    reset_state_timeout_ = parameter<double>(node_, "reset_state_timeout", 2.0);
    reset_joint_tolerance_ = parameter<double>(node_, "reset_joint_tolerance", 0.02);
    execution_settle_timeout_ = parameter<double>(
      node_, "execution_settle_timeout", reset_state_timeout_);
    execution_joint_tolerance_ = parameter<double>(
      node_, "execution_joint_tolerance", reset_joint_tolerance_);
    execution_velocity_tolerance_ = parameter<double>(
      node_, "execution_velocity_tolerance", 0.01);
    execution_settle_samples_ = parameter<int>(node_, "execution_settle_samples", 3);
    execution_feedback_max_age_ = parameter<double>(
      node_, "execution_feedback_max_age", 0.25);
    execution_feedback_future_skew_ = parameter<double>(
      node_, "execution_feedback_future_skew", 0.10);
    arm_state_topic_ = parameter<std::string>(
      node_, "arm_state_topic", "/aima/hal/joint/arm/state");
    if (reset_preemption_timeout_ <= 0.0 || reset_state_timeout_ <= 0.0 ||
      reset_joint_tolerance_ < 0.0 || execution_settle_timeout_ <= 0.0 ||
      execution_joint_tolerance_ < 0.0 || execution_velocity_tolerance_ < 0.0 ||
      execution_settle_samples_ < 1 || execution_feedback_max_age_ <= 0.0 ||
      execution_feedback_future_skew_ < 0.0 || arm_state_topic_.empty())
    {
      throw std::runtime_error(
              "reset/execution timeouts and feedback age must be positive; tolerances must be "
              "nonnegative; arm_state_topic must not be empty");
    }
    if (initial_state_ != "empty" && initial_state_ != "unknown") {
      throw std::runtime_error("initial_state must be 'empty' or 'unknown'");
    }
    simulate_attachment_ = parameter<bool>(node_, "simulate_ideal_attachment", true);
    perception_source_ = parsePerception3dSource(
      parameter<std::string>(node_, "perception_3d_source", "none"));
    depth_filtered_cloud_topic_ = parameter<std::string>(
      node_, "depth_filtered_cloud_topic", "/x2/moveit/depth_filtered_cloud");
    lidar_filtered_cloud_topic_ = parameter<std::string>(
      node_, "lidar_filtered_cloud_topic", "/x2/moveit/lidar_filtered_cloud");
    octomap_clear_service_ = parameter<std::string>(
      node_, "octomap_clear_service", "/clear_octomap");
    octomap_refill_timeout_ = parameter<double>(node_, "octomap_refill_timeout", 3.0);
    octomap_post_clear_samples_ = parameter<int>(node_, "octomap_post_clear_samples", 2);
    maximum_filtered_cloud_age_ = parameter<double>(
      node_, "maximum_filtered_cloud_age", 1.0);
    maximum_filtered_cloud_future_skew_ = parameter<double>(
      node_, "maximum_filtered_cloud_future_skew", 0.10);
    octomap_processing_delay_ = parameter<double>(node_, "octomap_processing_delay", 0.10);
    if (octomap_refill_timeout_ <= 0.0 || octomap_post_clear_samples_ < 1 ||
      maximum_filtered_cloud_age_ <= 0.0 || maximum_filtered_cloud_future_skew_ < 0.0 ||
      octomap_processing_delay_ < 0.0)
    {
      throw std::runtime_error(
              "OctoMap timeout/age must be positive, sample count at least one, and future "
              "skew/delay nonnegative");
    }
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
    if (ik_attempts_per_candidate_ < 1 ||
      maximum_grasp_candidates_ < 1 || maximum_planning_candidates_ < 1 ||
      maximum_retry_candidates_ < 0 || grasp_search_timeout_ <= 0.0 ||
      planning_time_per_candidate_ <= 0.0 || pregrasp_planning_timeout_ <= 0.0 ||
      minimum_grasp_joint_margin_ < 0.0 ||
      closed_chain_position_tolerance_ < 0.0 || closed_chain_orientation_tolerance_ < 0.0 ||
      closed_chain_position_step_ <= 0.0 || closed_chain_orientation_step_ <= 0.0 ||
      closed_chain_ik_attempts_ < 1 || closed_chain_ik_timeout_ <= 0.0 ||
      closed_chain_beam_width_ < 1 || closed_chain_solutions_per_branch_ < 1 ||
      closed_chain_projection_limit_ < 1 || closed_chain_validation_position_step_ <= 0.0 ||
      closed_chain_validation_orientation_step_ <= 0.0 ||
      closed_chain_contact_position_error_ < 0.0 ||
      closed_chain_contact_orientation_error_ < 0.0 ||
      carry_search_timeout_ <= 0.0 || maximum_carry_candidates_ < 1 ||
      carry_search_x_range_ < 0.0 || carry_search_y_range_ < 0.0 ||
      carry_search_z_lower_ < 0.0 || carry_search_z_upper_ < 0.0 ||
      carry_search_orientation_tolerance_ < 0.0 ||
      (adaptive_place_position_tolerance_.array() < 0.0).any() ||
      adaptive_place_yaw_tolerance_ < 0.0 ||
      grasp_position_tolerance_ < 0.0 || grasp_orientation_tolerance_ < 0.0 ||
      pregrasp_distance_tolerance_ < 0.0 || alternate_face_alignment_tolerance_ < 0.0)
    {
      throw std::runtime_error("grasp search counts/timeouts must be positive and tolerances nonnegative");
    }
    closed_chain_planner_ = std::make_unique<ClosedChainPathPlanner>(ClosedChainPlannerConfig{
      static_cast<std::size_t>(closed_chain_beam_width_),
      static_cast<std::size_t>(closed_chain_solutions_per_branch_),
      static_cast<std::size_t>(closed_chain_projection_limit_),
      closed_chain_position_tolerance_, closed_chain_orientation_tolerance_,
      closed_chain_position_step_, closed_chain_orientation_step_,
      std::max(closed_chain_validation_position_step_, cartesian_step_),
      std::max(closed_chain_validation_orientation_step_, closed_chain_orientation_step_),
      max_joint_step_});
    const auto nominal_grasp = computeGraspGeometry(
      Eigen::Isometry3d::Identity(), dimensions_, 0.0, contact_height_offset_);
    held_box_to_left_contact_ = nominal_grasp.left_contact;
    held_box_to_right_contact_ = nominal_grasp.right_contact;
    const auto named_targets = move_group_.getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(), post_place_named_target_) == named_targets.end())
    {
      throw std::runtime_error(
              "post_place_named_target is not defined for planning_group: " +
              post_place_named_target_);
    }
    if (std::find(
        named_targets.begin(), named_targets.end(), reset_named_target_) == named_targets.end())
    {
      throw std::runtime_error(
              "reset_named_target is not defined for planning_group: " + reset_named_target_);
    }
    reset_target_values_ = move_group_.getNamedTargetValues(reset_named_target_);
    if (reset_target_values_.size() != dual_group->getVariableCount()) {
      throw std::runtime_error("reset_named_target must define every planning-group joint");
    }

    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", "x2_pick_place_scene_monitor");
    if (!scene_monitor_->getPlanningScene()) {
      throw std::runtime_error("failed to create MoveIt planning scene monitor");
    }
    scene_monitor_->startStateMonitor();
    scene_monitor_->startSceneMonitor();
    scene_monitor_->startWorldGeometryMonitor();

    if (usesDepth(perception_source_)) {
      depth_filtered_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        depth_filtered_cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr message) {
          recordPerceptionSample(*message, depth_sample_);
        });
    }
    if (usesLidar(perception_source_)) {
      lidar_filtered_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_filtered_cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
          recordPerceptionSample(*message, lidar_sample_);
        });
    }
    clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>(octomap_clear_service_);

    hal_arm_state_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
      arm_state_topic_, rclcpp::SensorDataQoS(),
      [this](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {
        const rclcpp::Time measurement_time(message->header.meas_stamp);
        const double measurement_age = (node_->now() - measurement_time).seconds();
        if (measurement_time.nanoseconds() <= 0 || !std::isfinite(measurement_age) ||
          measurement_age > execution_feedback_max_age_ ||
          measurement_age < -execution_feedback_future_skew_)
        {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Ignoring stale HAL arm feedback from %s (measurement age %.6f s)",
            arm_state_topic_.c_str(), measurement_age);
          return;
        }
        std::map<std::string, JointFeedback> feedback;
        for (const auto & joint : message->joints) {
          if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity)) {
            RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 5000,
              "Ignoring HAL arm feedback with a non-finite state for %s", joint.name.c_str());
            return;
          }
          feedback[joint.name] = JointFeedback{joint.position, joint.velocity};
        }
        if (feedback.empty()) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Ignoring empty HAL arm feedback from %s", arm_state_topic_.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> lock(hal_arm_feedback_mutex_);
          if (latest_hal_arm_feedback_.valid && !isNewerMeasurementSequence(
              message->header.sequence, latest_hal_arm_feedback_.measurement_sequence))
          {
            RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 5000,
              "Ignoring non-increasing HAL arm measurement sequence %u from %s",
              message->header.sequence, arm_state_topic_.c_str());
            return;
          }
          latest_hal_arm_feedback_ = HalArmFeedbackSnapshot{
            std::move(feedback), message->header.sequence, true};
        }
        hal_arm_feedback_generation_.fetch_add(1U, std::memory_order_release);
      });

    box_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      box_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_box_pose_ = *message;
        have_box_pose_ = true;
      });
    planning_scene_audit_sub_ = node_->create_subscription<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", 10,
      [this](const moveit_msgs::msg::PlanningScene::SharedPtr message) {
        for (const auto & object : message->world.collision_objects) {
          auditCollisionObject(object, "/planning_scene");
        }
        for (const auto & attached : message->robot_state.attached_collision_objects) {
          auditCollisionObject(attached.object, "/planning_scene attached object");
        }
      });
    planning_scene_world_audit_sub_ =
      node_->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
      "/planning_scene_world", 10,
      [this](const moveit_msgs::msg::PlanningSceneWorld::SharedPtr message) {
        for (const auto & object : message->collision_objects) {
          auditCollisionObject(object, "/planning_scene_world");
        }
      });
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/grasp_markers", 10);
    box_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/pick_place/planned_box_path", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pick_place/planning_diagnostics", 10);
    state_pub_ = node_->create_publisher<ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local());
    attach_client_ = node_->create_client<std_srvs::srv::Trigger>(attach_service);
    detach_client_ = node_->create_client<std_srvs::srv::Trigger>(detach_service);
    execute_trajectory_client_ = rclcpp_action::create_client<ExecuteTrajectory>(
      node_, "execute_trajectory");

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
    reset_action_server_ = rclcpp_action::create_server<ResetManipulation>(
      node_, "reset_manipulation",
      std::bind(&PickPlaceServer::onResetGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onResetCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onResetAccepted, this, std::placeholders::_1));
    recovery_service_ = node_->create_service<RecoverManipulationState>(
      "/recover_manipulation_state",
      std::bind(
        &PickPlaceServer::recoverState, this, std::placeholders::_1, std::placeholders::_2));
    publishState();
  }

private:
  static const moveit_msgs::msg::RobotTrajectory & executableTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    return plan.trajectory_;
  }

  static const moveit_msgs::msg::RobotTrajectory & executableTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    return trajectory;
  }

  void setExecutionError(const std::string & error)
  {
    std::lock_guard<std::mutex> lock(execution_error_mutex_);
    last_execution_error_ = error;
  }

  std::string executionError(const std::string & prefix)
  {
    std::lock_guard<std::mutex> lock(execution_error_mutex_);
    return last_execution_error_.empty() ? prefix : prefix + ": " + last_execution_error_;
  }

  bool waitForExecutionSettled(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const HalArmFeedbackSnapshot & prior_feedback, uint64_t minimum_feedback_generation,
    const CancelFunction & canceled, std::map<std::string, double> * settled_positions)
  {
    const auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty()) {
      setExecutionError("controller reported success for an empty joint trajectory");
      return false;
    }
    const auto & endpoint = joint_trajectory.points.back();
    if (endpoint.positions.size() != joint_trajectory.joint_names.size()) {
      setExecutionError("trajectory endpoint does not contain every commanded joint position");
      return false;
    }
    std::map<std::string, double> target_positions;
    for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index) {
      target_positions[joint_trajectory.joint_names[index]] = endpoint.positions[index];
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(execution_settle_timeout_));
    uint64_t last_feedback_generation = minimum_feedback_generation;
    int settled_samples = 0;
    std::string worst_position_joint{"unavailable"};
    std::string worst_velocity_joint{"unavailable"};
    double maximum_position_error = std::numeric_limits<double>::infinity();
    double maximum_velocity = std::numeric_limits<double>::infinity();
    while (std::chrono::steady_clock::now() < deadline) {
      if (execution_cancel_requested_.load() || canceled()) {
        setExecutionError("execution canceled while waiting for fresh settled joint feedback");
        return false;
      }
      const uint64_t feedback_generation = hal_arm_feedback_generation_.load(
        std::memory_order_acquire);
      if (feedback_generation <= last_feedback_generation) {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      last_feedback_generation = feedback_generation;
      HalArmFeedbackSnapshot feedback;
      {
        std::lock_guard<std::mutex> lock(hal_arm_feedback_mutex_);
        feedback = latest_hal_arm_feedback_;
      }
      if (!feedback.valid || !isNewerMeasurementSequence(
          feedback.measurement_sequence, prior_feedback.measurement_sequence))
      {
        settled_samples = 0;
        continue;
      }
      const auto check = checkExecutionFeedback(
        target_positions, feedback.joints, execution_joint_tolerance_, execution_velocity_tolerance_);
      maximum_position_error = check.maximum_position_error;
      maximum_velocity = check.maximum_velocity;
      worst_position_joint = check.worst_position_joint;
      worst_velocity_joint = check.worst_velocity_joint;
      settled_samples = check.settled ? settled_samples + 1 : 0;
      if (settled_samples >= execution_settle_samples_) {
        if (settled_positions) {
          settled_positions->clear();
          for (const auto & [joint_name, target_position] : target_positions) {
            (void)target_position;
            (*settled_positions)[joint_name] = feedback.joints.at(joint_name).position;
          }
        }
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream message;
    message << std::fixed << std::setprecision(6) <<
      "execution endpoint did not settle within " << execution_settle_timeout_ <<
      " s (worst_position_joint=" << worst_position_joint <<
      ", position_error=" << maximum_position_error << " rad, worst_velocity_joint=" <<
      worst_velocity_joint << ", velocity=" << maximum_velocity << " rad/s, samples=" <<
      settled_samples << "/" << execution_settle_samples_ << ")";
    setExecutionError(message.str());
    RCLCPP_ERROR(node_->get_logger(), "%s", message.str().c_str());
    move_group_.stop();
    return false;
  }

  template<typename TrajectoryT>
  bool executeMotion(
    const TrajectoryT & trajectory, const CancelFunction & canceled,
    std::map<std::string, double> * settled_positions = nullptr)
  {
    setExecutionError("");
    if (!execute_trajectory_client_->wait_for_action_server(std::chrono::seconds(2))) {
      setExecutionError("ExecuteTrajectory action server is unavailable");
      return false;
    }
    ExecuteTrajectory::Goal request;
    request.trajectory = executableTrajectory(trajectory);
    std::shared_future<ExecuteTrajectoryGoalHandle::SharedPtr> goal_future;
    {
      std::lock_guard<std::mutex> lock(execution_transition_mutex_);
      if (execution_cancel_requested_.load() || canceled()) {
        return false;
      }
      goal_future = execute_trajectory_client_->async_send_goal(request);
    }

    while (goal_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (execution_cancel_requested_.load() || canceled()) {
        move_group_.stop();
      }
    }
    const auto handle = goal_future.get();
    if (!handle) {
      setExecutionError("trajectory goal was rejected");
      return false;
    }
    bool cancel_now = false;
    {
      std::lock_guard<std::mutex> lock(execution_transition_mutex_);
      active_execution_goal_ = handle;
      cancel_now = execution_cancel_requested_.load() || canceled();
    }
    if (cancel_now) {
      (void)execute_trajectory_client_->async_cancel_goal(handle);
    }

    auto result_future = execute_trajectory_client_->async_get_result(handle);
    while (result_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (execution_cancel_requested_.load() || canceled()) {
        (void)execute_trajectory_client_->async_cancel_goal(handle);
      }
    }
    const auto result = result_future.get();
    {
      std::lock_guard<std::mutex> lock(execution_transition_mutex_);
      if (active_execution_goal_ == handle) {
        active_execution_goal_.reset();
      }
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      setExecutionError("MoveIt ExecuteTrajectory action did not report success");
      return false;
    }
    HalArmFeedbackSnapshot prior_feedback;
    {
      std::lock_guard<std::mutex> lock(hal_arm_feedback_mutex_);
      prior_feedback = latest_hal_arm_feedback_;
    }
    if (!prior_feedback.valid) {
      setExecutionError("no valid HAL arm measurement was available after trajectory execution");
      return false;
    }
    const uint64_t feedback_generation = hal_arm_feedback_generation_.load(
      std::memory_order_acquire);
    return waitForExecutionSettled(
      executableTrajectory(trajectory), prior_feedback, feedback_generation, canceled,
      settled_positions);
  }

  void requestExecutionStop()
  {
    execution_cancel_requested_.store(true);
    ExecuteTrajectoryGoalHandle::SharedPtr active_goal;
    {
      std::lock_guard<std::mutex> lock(execution_transition_mutex_);
      active_goal = active_execution_goal_;
    }
    if (active_goal) {
      (void)execute_trajectory_client_->async_cancel_goal(active_goal);
    }
    move_group_.stop();
  }

  void auditCollisionObject(
    const moveit_msgs::msg::CollisionObject & object, const char * topic) const
  {
    if (object.operation == moveit_msgs::msg::CollisionObject::REMOVE ||
      object.header.frame_id.empty() || object.header.frame_id == planning_frame_)
    {
      return;
    }
    if (object.id == box_id_) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Planning-scene box '%s' arrived on %s in frame '%s'; expected '%s'. "
        "Stop the conflicting publisher; pick_place_server recreates this object.",
        object.id.c_str(), topic, object.header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "External collision object '%s' arrived on %s in frame '%s'. Its publisher must "
      "provide timestamp-compatible TF to planning frame '%s'.",
      object.id.c_str(), topic, object.header.frame_id.c_str(), planning_frame_.c_str());
  }

  template<typename MessageT>
  void recordPerceptionSample(const MessageT & message, PerceptionSample & sample)
  {
    std::lock_guard<std::mutex> lock(perception_mutex_);
    ++sample.count;
    sample.header_nanoseconds = rclcpp::Time(message.header.stamp).nanoseconds();
    sample.receipt_nanoseconds = node_->now().nanoseconds();
    perception_condition_.notify_all();
  }

  bool clearOctomap(std::string & error)
  {
    const auto timeout = std::chrono::duration<double>(octomap_refill_timeout_);
    try {
      if (!clear_octomap_client_->wait_for_service(timeout)) {
        error = "OctoMap clear service is unavailable: " + octomap_clear_service_;
        return false;
      }
      auto future = clear_octomap_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
      if (future.wait_for(timeout) != std::future_status::ready) {
        error = "OctoMap clear service timed out: " + octomap_clear_service_;
        return false;
      }
      future.get();
    } catch (const std::exception & exception) {
      error = "OctoMap clear request failed: " + std::string(exception.what());
      return false;
    }
    return true;
  }

  bool synchronizePlanningScene(std::string & error)
  {
    try {
      if (!scene_monitor_->requestPlanningSceneState("/get_planning_scene")) {
        error = "failed to synchronize the refreshed MoveIt planning scene";
        return false;
      }
      return true;
    } catch (const std::exception & exception) {
      error = "planning-scene synchronization failed: " + std::string(exception.what());
      return false;
    }
  }

  bool refreshOctomap(std::string & error)
  {
    if (perception_source_ == Perception3dSource::NONE) {
      return true;
    }
    if (!clearOctomap(error)) {
      return false;
    }

    const auto timeout = std::chrono::duration<double>(octomap_refill_timeout_);
    std::unique_lock<std::mutex> lock(perception_mutex_);
    const PerceptionSnapshot snapshot{depth_sample_.count, lidar_sample_.count};
    const auto ready = [this, &snapshot]() {
        return perceptionReady(
          perception_source_, depth_sample_, lidar_sample_, snapshot,
          static_cast<uint64_t>(octomap_post_clear_samples_), node_->now().nanoseconds(),
          static_cast<int64_t>(maximum_filtered_cloud_age_ * 1e9),
          static_cast<int64_t>(maximum_filtered_cloud_future_skew_ * 1e9));
      };
    if (!perception_condition_.wait_for(lock, timeout, ready)) {
      std::string sources;
      if (usesDepth(perception_source_)) {
        sources += "depth";
      }
      if (usesLidar(perception_source_)) {
        sources += sources.empty() ? "lidar" : " and lidar";
      }
      error = "no fresh post-clear filtered cloud samples from " + sources +
        " (check topic remapping, timestamps, CameraInfo, and sensor-to-base_link TF)";
      return false;
    }
    lock.unlock();

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(octomap_processing_delay_)));
    if (!synchronizePlanningScene(error)) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "OctoMap cleared and refilled with %d post-clear sample(s)",
      octomap_post_clear_samples_);
    return true;
  }

  bool reserveGoal()
  {
    return reset_coordinator_.reserveOperation();
  }

  void releaseOperation()
  {
    reset_coordinator_.releaseOperation();
  }

  rclcpp_action::GoalResponse onPickGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal)
  {
    if (!goal->plan_only && !allow_execution_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting Pick execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    execution_cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal> goal)
  {
    if (!goal->plan_only && !allow_execution_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting Place execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    execution_cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onPickPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (!goal->plan_only && !allow_execution_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting PickPlace execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    execution_cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onResetGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ResetManipulation::Goal> goal)
  {
    if (!allow_execution_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting reset motion: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->confirm_empty) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    if (!reset_coordinator_.requestReset()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    requestExecutionStop();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse cancelGoal(const std::shared_ptr<GoalHandleT> &)
  {
    requestExecutionStop();
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

  rclcpp_action::CancelResponse onResetCancel(const std::shared_ptr<ResetGoalHandle> & goal)
  {
    const auto response = cancelGoal(goal);
    reset_coordinator_.notify();
    return response;
  }

  void onPickAccepted(const std::shared_ptr<PickGoalHandle> goal)
  {
    std::thread([this, goal]() {executePick(goal);}).detach();
  }

  void onPlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {executePlace(goal);}).detach();
  }

  void onPickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {executePickPlace(goal);}).detach();
  }

  void onResetAccepted(const std::shared_ptr<ResetGoalHandle> goal)
  {
    if (!goal->get_goal()->confirm_empty) {
      auto result = std::make_shared<ResetManipulation::Result>();
      result->success = false;
      result->error_code = ResetManipulation::Result::CONFIRMATION_REQUIRED;
      result->message = "confirm_empty must be true after the operator verifies that no object is held";
      goal->abort(result);
      return;
    }
    std::thread([this, goal]() {
      try {
        executeReset(goal);
      } catch (const std::exception & exception) {
        const std::string message =
          "reset failed with unhandled exception: " + std::string(exception.what());
        setState(ManipulationState::RECOVERY_REQUIRED, message);
        reset_coordinator_.finishReset(false);
        auto result = std::make_shared<ResetManipulation::Result>();
        result->success = false;
        result->error_code = ResetManipulation::Result::CLEANUP_FAILED;
        result->message = message;
        if (goal->is_canceling()) {
          result->error_code = ResetManipulation::Result::CANCELED;
          goal->canceled(result);
        } else {
          goal->abort(result);
        }
      }
    }).detach();
  }

  void initializeState()
  {
    std::ifstream input(state_file_);
    std::string saved;
    input >> saved;
    if (saved == "VERSION") {
      int version = 0;
      std::string label;
      input >> version >> label >> saved;
      if (version == 2 && label == "STATE" && saved == "HOLDING") {
        Eigen::Isometry3d pose;
        Eigen::Isometry3d left_contact;
        Eigen::Isometry3d right_contact;
        std::string pose_label;
        std::string left_label;
        std::string right_label;
        input >> pose_label;
        const bool pose_ok = pose_label == "POSE" && readTransform(input, pose);
        input >> left_label;
        const bool left_ok = left_label == "LEFT_CONTACT" && readTransform(input, left_contact);
        input >> right_label;
        const bool right_ok = right_label == "RIGHT_CONTACT" &&
          readTransform(input, right_contact);
        if (pose_ok && left_ok && right_ok) {
          held_pose_ = stampedPose(pose);
          held_box_to_left_contact_ = left_contact;
          held_box_to_right_contact_ = right_contact;
          held_geometry_valid_ = true;
        } else {
          RCLCPP_WARN(
            node_->get_logger(),
            "Ignoring incomplete persisted holding geometry in '%s'", state_file_.c_str());
        }
      }
    }
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
        output << "VERSION 2\nSTATE " <<
          (state == ManipulationState::EMPTY ? "EMPTY\n" : "HOLDING\n");
        if (state != ManipulationState::EMPTY && held_geometry_valid_ &&
          !held_pose_.header.frame_id.empty())
        {
          output << "POSE ";
          writeTransform(output, toEigen(held_pose_.pose));
          output << "LEFT_CONTACT ";
          writeTransform(output, held_box_to_left_contact_);
          output << "RIGHT_CONTACT ";
          writeTransform(output, held_box_to_right_contact_);
        }
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

  void setState(
    uint8_t state, const std::string & detail, bool persist = true,
    bool override_reset_latch = false)
  {
    if (reset_coordinator_.resetRequested() && state != ManipulationState::RECOVERY_REQUIRED &&
      !override_reset_latch)
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_detail_ = detail;
    }
    state_.store(state);
    if (state == ManipulationState::EMPTY) {
      held_geometry_valid_ = false;
    }
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
    Eigen::Isometry3d recovered_pose;
    if (held_geometry_valid_) {
      const Eigen::Isometry3d left_estimate =
        current->getGlobalLinkTransform(left_tcp_) * held_box_to_left_contact_.inverse();
      const Eigen::Isometry3d right_estimate =
        current->getGlobalLinkTransform(right_tcp_) * held_box_to_right_contact_.inverse();
      if (!within_tolerance(left_estimate, right_estimate)) {
        error = "left/right TCPs do not imply the same persisted box pose within recovery tolerances";
        return false;
      }
      recovered_pose = left_estimate;
      recovered_pose.translation() =
        0.5 * (left_estimate.translation() + right_estimate.translation());
      Eigen::Quaterniond left_rotation(left_estimate.linear());
      Eigen::Quaterniond right_rotation(right_estimate.linear());
      if (left_rotation.dot(right_rotation) < 0.0) {
        right_rotation.coeffs() *= -1.0;
      }
      recovered_pose.linear() = left_rotation.slerp(0.5, right_rotation).normalized().toRotationMatrix();
    } else {
      const auto grasp = computeGraspGeometry(
        carry_pose_, dimensions_, 0.0, contact_height_offset_);
      if (!within_tolerance(current->getGlobalLinkTransform(left_tcp_), grasp.left_contact) ||
        !within_tolerance(current->getGlobalLinkTransform(right_tcp_), grasp.right_contact))
      {
        error = "TCP poses do not match the configured legacy carry pose within recovery tolerances";
        return false;
      }
      recovered_pose = carry_pose_;
      held_box_to_left_contact_ = carry_pose_.inverse() * grasp.left_contact;
      held_box_to_right_contact_ = carry_pose_.inverse() * grasp.right_contact;
      held_geometry_valid_ = true;
    }
    if (!clearBoxScene(error) || !applyBox(recovered_pose, error)) {
      return false;
    }
    if (!attachBoxToScene(error)) {
      return false;
    }
    physical_attachment_expected_.store(simulate_attachment_);
    held_pose_ = stampedPose(recovered_pose);
    return true;
  }

  void recoverState(
    const std::shared_ptr<RecoverManipulationState::Request> request,
    std::shared_ptr<RecoverManipulationState::Response> response)
  {
    if (reset_coordinator_.resetRequested() || reset_coordinator_.resetPending()) {
      response->message = "manipulation reset is pending";
      return;
    }
    if (!reserveGoal()) {
      response->message = "manipulation server is busy";
      return;
    }
    ScopeExit release([this]() {releaseOperation();});
    try {
      if (request->requested_state == RecoverManipulationState::Request::CONFIRM_EMPTY) {
        std::string error;
        if (!clearBoxScene(error)) {
          response->message = error;
          return;
        }
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
    } catch (const std::exception & exception) {
      response->message = "manipulation recovery failed: " + std::string(exception.what());
    }
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

  bool boxPoseStillWithinTolerance(
    const Eigen::Isometry3d & reference, geometry_msgs::msg::PoseStamped & latest,
    std::string & error)
  {
    if (!stableBoxPose(latest)) {
      error = "box pose became stale before approach";
      return false;
    }
    Eigen::Isometry3d current;
    try {
      current = toEigen(latest.pose);
    } catch (const std::exception & exception) {
      error = exception.what();
      return false;
    }
    const double position_error = (current.translation() - reference.translation()).norm();
    const Eigen::Quaterniond reference_q(reference.linear());
    const Eigen::Quaterniond current_q(current.linear());
    const double angular_error = 2.0 * std::acos(
      std::clamp(std::abs(reference_q.dot(current_q)), 0.0, 1.0));
    if (position_error > grasp_position_tolerance_ ||
      angular_error > grasp_orientation_tolerance_)
    {
      error = "box moved after planning (position=" + std::to_string(position_error) +
        " m, angle=" + std::to_string(angular_error) + " rad)";
      return false;
    }
    return true;
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

  bool applyBox(const Eigen::Isometry3d & pose, std::string & error)
  {
    try {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = planning_frame_;
      object.header.stamp.sec = 0;
      object.header.stamp.nanosec = 0;
      object.id = box_id_;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {dimensions_.length, dimensions_.width, dimensions_.height};
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(toPoseMsg(pose));
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      if (!planning_scene_interface_.applyCollisionObject(object)) {
        error = "MoveIt rejected the box collision object";
        return false;
      }
      return true;
    } catch (const std::exception & exception) {
      error = "failed to apply the box collision object: " + std::string(exception.what());
      return false;
    }
  }

  bool removeBox(std::string & error)
  {
    try {
      if (planning_scene_interface_.getObjects({box_id_}).empty()) {
        return true;
      }
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = planning_frame_;
      object.id = box_id_;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_interface_.applyCollisionObject(object)) {
        error = "MoveIt rejected removal of the box collision object";
        return false;
      }
      return true;
    } catch (const std::exception & exception) {
      error = "failed to remove the box collision object: " + std::string(exception.what());
      return false;
    }
  }

  bool detachBoxFromScene(std::string & error)
  {
    try {
      if (planning_scene_interface_.getAttachedObjects({box_id_}).empty()) {
        return true;
      }
      moveit_msgs::msg::AttachedCollisionObject object;
      object.object.id = box_id_;
      object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_interface_.applyAttachedCollisionObject(object)) {
        error = "MoveIt rejected box detachment from the planning scene";
        return false;
      }
      return true;
    } catch (const std::exception & exception) {
      error = "failed to detach the planning-scene box: " + std::string(exception.what());
      return false;
    }
  }

  bool attachBoxToScene(std::string & error)
  {
    try {
      moveit_msgs::msg::AttachedCollisionObject object;
      object.link_name = left_tcp_;
      object.object.id = box_id_;
      object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
      object.touch_links = {
        "left_wrist_roll_link", "left_hand_pad_link", left_tcp_,
        "right_wrist_roll_link", "right_hand_pad_link", right_tcp_};
      if (!planning_scene_interface_.applyAttachedCollisionObject(object)) {
        error = "MoveIt rejected box attachment to the planning scene";
        return false;
      }
      return verifyBoxSceneState(true, false, error);
    } catch (const std::exception & exception) {
      error = "failed to attach the planning-scene box: " + std::string(exception.what());
      return false;
    }
  }

  bool verifyBoxSceneState(bool expect_attached, bool expect_world, std::string & error)
  {
    try {
      const auto attached_objects = planning_scene_interface_.getAttachedObjects({box_id_});
      const auto world_objects = planning_scene_interface_.getObjects({box_id_});
      const bool attached = !attached_objects.empty();
      const bool world = !world_objects.empty();
      if (attached != expect_attached || world != expect_world) {
        error = "planning-scene box state mismatch (attached=" +
          std::string(attached ? "true" : "false") + ", world=" +
          std::string(world ? "true" : "false") + ")";
        return false;
      }
      if (expect_world) {
        const auto object = world_objects.find(box_id_);
        if (object == world_objects.end() || object->second.header.frame_id != planning_frame_) {
          error = "planning-scene box is not expressed in planning frame " + planning_frame_;
          return false;
        }
        const auto & stamp = object->second.header.stamp;
        if (stamp.sec != 0 || stamp.nanosec != 0U) {
          error = "planning-scene box has a nonzero transform timestamp";
          return false;
        }
      }
      return true;
    } catch (const std::exception & exception) {
      error = "failed to verify the planning-scene box: " + std::string(exception.what());
      return false;
    }
  }

  bool clearBoxScene(std::string & error)
  {
    return detachBoxFromScene(error) && removeBox(error) &&
           verifyBoxSceneState(false, false, error);
  }

  bool placeBoxInScene(const Eigen::Isometry3d & pose, std::string & error)
  {
    return detachBoxFromScene(error) && applyBox(pose, error) &&
           verifyBoxSceneState(false, true, error);
  }

  bool callAttachmentService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, std::string & error,
    bool * request_dispatched = nullptr)
  {
    if (request_dispatched) {
      *request_dispatched = false;
    }
    if (!simulate_attachment_) {
      return true;
    }
    try {
      if (!client->wait_for_service(std::chrono::seconds(2))) {
        error = "MuJoCo attachment service is unavailable";
        return false;
      }
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
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

  bool collisionFree(moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    if (ignore_box) {
      acm.setEntry(box_id_, true);
    } else if (allow_pad_contact) {
      acm.setEntry(box_id_, std::vector<std::string>{
        "left_wrist_roll_link", "left_hand_pad_link", left_tcp_,
        "right_wrist_roll_link", "right_hand_pad_link", right_tcp_}, true);
    }
    collision_detection::CollisionRequest request;
    request.group_name = move_group_.getName();
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state, acm);
    return !result.collision;
  }

  bool collisionFreeWithBox(
    moveit::core::RobotState & state, const Eigen::Isometry3d & box_pose,
    bool allow_pad_contact)
  {
    planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
    auto scene = locked->diff();
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = planning_frame_;
    object.id = box_id_;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {dimensions_.length, dimensions_.width, dimensions_.height};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(toPoseMsg(box_pose));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene->processCollisionObjectMsg(object)) {
      return false;
    }
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    if (allow_pad_contact) {
      acm.setEntry(box_id_, std::vector<std::string>{
        "left_wrist_roll_link", "left_hand_pad_link", left_tcp_,
        "right_wrist_roll_link", "right_hand_pad_link", right_tcp_}, true);
    }
    collision_detection::CollisionRequest request;
    request.group_name = move_group_.getName();
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state, acm);
    return !result.collision;
  }

  bool setFromDualArmIK(
    moveit::core::RobotState & state, const Eigen::Isometry3d & left_pose,
    const Eigen::Isometry3d & right_pose, bool require_continuity = false,
    double timeout = -1.0) const
  {
    const auto * left_group = state.getJointModelGroup(left_group_name_);
    const auto * right_group = state.getJointModelGroup(right_group_name_);
    const std::vector<double> left_consistency(
      left_group->getVariableCount(), max_joint_step_);
    const std::vector<double> right_consistency(
      right_group->getVariableCount(), max_joint_step_);
    const double solve_timeout = timeout > 0.0 ? timeout : ik_timeout_;
    state.update();
    const bool left_solved = require_continuity ?
      state.setFromIK(left_group, left_pose, left_tcp_, left_consistency, solve_timeout) :
      state.setFromIK(left_group, left_pose, left_tcp_, solve_timeout);
    if (!left_solved) {
      return false;
    }
    state.update();
    const bool right_solved = require_continuity ?
      state.setFromIK(right_group, right_pose, right_tcp_, right_consistency, solve_timeout) :
      state.setFromIK(right_group, right_pose, right_tcp_, solve_timeout);
    if (!right_solved) {
      return false;
    }
    state.update();
    return true;
  }

  bool solvePregraspIK(
    const moveit::core::RobotState & seed, const GraspGeometry & grasp,
    int attempts, moveit::core::RobotState & solution, double & joint_limit_margin,
    double & joint_distance, std::string & error)
  {
    const auto * dual_group = seed.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounded = false;
    bool found_collision_free = false;
    double best_margin = -std::numeric_limits<double>::infinity();
    double best_distance = std::numeric_limits<double>::infinity();
    moveit::core::RobotState best(seed);

    for (int attempt = 0; attempt < attempts; ++attempt) {
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
      const double margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
      if (margin > best_margin + 1e-12 ||
        (std::abs(margin - best_margin) <= 1e-12 && distance < best_distance))
      {
        best_margin = margin;
        best_distance = distance;
        best = candidate;
      }
    }

    if (found_collision_free) {
      solution = best;
      solution.update();
      joint_limit_margin = best_margin;
      joint_distance = best_distance;
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

  GraspGeometry graspFromBoxToTcp(
    const Eigen::Isometry3d & box_pose, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, double pregrasp_distance) const
  {
    GraspGeometry grasp;
    grasp.left_contact = box_pose * box_to_left_contact;
    grasp.right_contact = box_pose * box_to_right_contact;
    grasp.left_outward_normal =
      (grasp.left_contact.linear() * Eigen::Vector3d::UnitY()).normalized();
    grasp.right_outward_normal =
      (grasp.right_contact.linear() * -Eigen::Vector3d::UnitY()).normalized();
    grasp.left_pregrasp = grasp.left_contact;
    grasp.right_pregrasp = grasp.right_contact;
    grasp.left_pregrasp.translation() += grasp.left_outward_normal * pregrasp_distance;
    grasp.right_pregrasp.translation() += grasp.right_outward_normal * pregrasp_distance;
    return grasp;
  }

  std::vector<Eigen::Isometry3d> sharedPoseCandidates(
    const Eigen::Isometry3d & nominal, bool allow_projection) const
  {
    std::vector<Eigen::Isometry3d> poses{nominal};
    if (!allow_projection) {
      return poses;
    }
    for (double magnitude = closed_chain_position_step_;
      magnitude <= closed_chain_position_tolerance_ + 1e-9;
      magnitude += closed_chain_position_step_)
    {
      for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
          auto pose = nominal;
          pose.translation()[axis] += sign * magnitude;
          poses.push_back(pose);
        }
      }
    }
    for (double angle = closed_chain_orientation_step_;
      angle <= closed_chain_orientation_tolerance_ + 1e-9;
      angle += closed_chain_orientation_step_)
    {
      for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
          auto pose = nominal;
          pose.linear() = pose.linear() *
            Eigen::AngleAxisd(sign * angle, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
          poses.push_back(pose);
        }
      }
    }
    return poses;
  }

  static double poseAngularError(
    const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected)
  {
    const Eigen::Quaterniond qa(actual.linear());
    const Eigen::Quaterniond qb(expected.linear());
    return 2.0 * std::acos(std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
  }

  void publishPlanningDiagnostic(
    const ClosedChainSearchReport & report, const std::string & route,
    const Eigen::Isometry3d & nominal, const Eigen::Isometry3d & realized,
    double joint_margin, double maximum_joint_step)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pick_place/closed_chain_planning";
    status.hardware_id = "x2_dual_arm";
    status.level = report.failure == ClosedChainFailure::NONE ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = report.detail.empty() ? closedChainFailureName(report.failure) : report.detail;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    const auto poseString = [](const Eigen::Isometry3d & pose) {
        const Eigen::Quaterniond q(pose.linear());
        std::ostringstream stream;
        stream << std::setprecision(9) << pose.translation().transpose() << " " <<
          q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        return stream.str();
      };
    add("grasp_id", active_grasp_id_);
    add("route_id", route);
    add("segment", report.segment);
    add("waypoint", std::to_string(report.waypoint));
    add("nominal_box_pose", poseString(nominal));
    add("realized_box_pose", poseString(realized));
    add("projection_offset", poseString(nominal.inverse() * realized));
    add("classification", closedChainFailureName(report.failure));
    add("failed_arm", report.failed_arm);
    add("ik_mode", report.ik_mode);
    add("attempts", std::to_string(report.attempts));
    add("elapsed_budget", std::to_string(report.elapsed) + "/" +
      std::to_string(carry_search_timeout_));
    add("joint_margin", std::to_string(joint_margin));
    add("maximum_joint_step", std::to_string(maximum_joint_step));
    diagnostics_pub_->publish(array);
  }

  bool appendObjectPath(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const std::vector<ClosedChainWaypoint> & controls, bool ignore_box,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, const std::string & route,
    const std::chrono::steady_clock::time_point & deadline, const CancelFunction & canceled,
    std::string & error)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    std::vector<double> start_joints;
    state.copyJointGroupPositions(dual_group, start_joints);
    const moveit::core::RobotState fixed_state(state);
    ClosedChainPath path;
    ClosedChainSearchReport report;
    const ClosedChainSolve solve =
      [this, &fixed_state, dual_group, ignore_box, &box_to_left_contact,
        &box_to_right_contact](
        const std::vector<double> & seed, const Eigen::Isometry3d & from_box_pose,
        const Eigen::Isometry3d & box_pose,
        std::size_t solution_limit, const std::chrono::steady_clock::time_point & solve_deadline,
        const ClosedChainCancel & solve_canceled, ClosedChainSearchReport & solve_report) {
        std::vector<ClosedChainSolution> solutions;
        const auto grasp = graspFromBoxToTcp(
          box_pose, box_to_left_contact, box_to_right_contact, 0.0);
        for (int attempt = 0; attempt < closed_chain_ik_attempts_ &&
          solutions.size() < solution_limit; ++attempt)
        {
          if (solve_canceled()) {
            solve_report.failure = ClosedChainFailure::CANCELED;
            break;
          }
          const auto now = std::chrono::steady_clock::now();
          if (now >= solve_deadline) {
            solve_report.failure = ClosedChainFailure::DEADLINE;
            break;
          }
          moveit::core::RobotState candidate(fixed_state);
          candidate.setJointGroupPositions(dual_group, seed);
          candidate.update();
          if (attempt > 0) {
            candidate.setToRandomPositions(dual_group);
          }
          const auto * left_group = candidate.getJointModelGroup(left_group_name_);
          const auto * right_group = candidate.getJointModelGroup(right_group_name_);
          const double remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double timeout = std::max(1e-4, std::min(closed_chain_ik_timeout_, remaining));
          const std::vector<double> left_consistency(left_group->getVariableCount(), max_joint_step_);
          const bool left_ok = attempt == 0 ?
            candidate.setFromIK(
            left_group, grasp.left_contact, left_tcp_, left_consistency, timeout) :
            candidate.setFromIK(left_group, grasp.left_contact, left_tcp_, timeout);
          if (!left_ok) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "left arm IK failed";
            solve_report.failed_arm = "left";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (solve_canceled()) {
            solve_report.failure = ClosedChainFailure::CANCELED;
            break;
          }
          if (std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = ClosedChainFailure::DEADLINE;
            break;
          }
          const std::vector<double> right_consistency(
            right_group->getVariableCount(), max_joint_step_);
          const double right_remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double right_timeout = std::max(
            1e-4, std::min(closed_chain_ik_timeout_, right_remaining));
          const bool right_ok = attempt == 0 ?
            candidate.setFromIK(
            right_group, grasp.right_contact, right_tcp_, right_consistency, right_timeout) :
            candidate.setFromIK(right_group, grasp.right_contact, right_tcp_, right_timeout);
          if (!right_ok) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "right arm IK failed";
            solve_report.failed_arm = "right";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (!candidate.satisfiesBounds(dual_group)) {
            solve_report.failure = ClosedChainFailure::BOUNDS;
            solve_report.detail = "dual-arm joint bounds violated";
            continue;
          }
          std::vector<double> joints;
          candidate.copyJointGroupPositions(dual_group, joints);
          double largest_step = 0.0;
          for (const auto * joint : dual_group->getActiveJointModels()) {
            moveit::core::RobotState predecessor(fixed_state);
            predecessor.setJointGroupPositions(dual_group, seed);
            predecessor.update();
            largest_step = std::max(largest_step, joint->distance(
              predecessor.getJointPositions(joint), candidate.getJointPositions(joint)));
          }
          if (largest_step > max_joint_step_) {
            solve_report.failure = ClosedChainFailure::CONTINUITY;
            solve_report.detail = "maximum joint step exceeded";
            continue;
          }
          const auto & actual_left = candidate.getGlobalLinkTransform(left_tcp_);
          const auto & actual_right = candidate.getGlobalLinkTransform(right_tcp_);
          if ((actual_left.translation() - grasp.left_contact.translation()).norm() >
            closed_chain_contact_position_error_ ||
            (actual_right.translation() - grasp.right_contact.translation()).norm() >
            closed_chain_contact_position_error_ ||
            poseAngularError(actual_left, grasp.left_contact) >
            closed_chain_contact_orientation_error_ ||
            poseAngularError(actual_right, grasp.right_contact) >
            closed_chain_contact_orientation_error_)
          {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "rigid TCP-to-box contact closure violated";
            continue;
          }
          const bool collision_free = ignore_box ?
            collisionFreeWithBox(candidate, box_pose, true) :
            collisionFree(candidate, true, false);
          if (!collision_free) {
            solve_report.failure = ClosedChainFailure::COLLISION;
            solve_report.detail = "state or held box is in collision";
            continue;
          }
          bool edge_valid = true;
          const int edge_samples = std::max(2, static_cast<int>(std::ceil(std::max({
            largest_step / std::max(1e-6, max_joint_step_ * 0.5),
            (box_pose.translation() - from_box_pose.translation()).norm() /
            closed_chain_validation_position_step_,
            poseAngularError(from_box_pose, box_pose) /
            closed_chain_validation_orientation_step_}))));
          moveit::core::RobotState predecessor(fixed_state);
          predecessor.setJointGroupPositions(dual_group, seed);
          predecessor.update();
          for (int sample = 1; sample < edge_samples; ++sample) {
            const double t = static_cast<double>(sample) / edge_samples;
            moveit::core::RobotState interpolated(fixed_state);
            predecessor.interpolate(candidate, t, interpolated);
            interpolated.update();
            const Eigen::Isometry3d expected_box = interpolatePose(
              from_box_pose, box_pose, t);
            const auto expected_grasp = graspFromBoxToTcp(
              expected_box, box_to_left_contact, box_to_right_contact, 0.0);
            if (!interpolated.satisfiesBounds(dual_group) ||
              (ignore_box ? !collisionFreeWithBox(interpolated, expected_box, true) :
              !collisionFree(interpolated, true, false)) ||
              (interpolated.getGlobalLinkTransform(left_tcp_).translation() -
              expected_grasp.left_contact.translation()).norm() >
              closed_chain_contact_position_error_ ||
              (interpolated.getGlobalLinkTransform(right_tcp_).translation() -
              expected_grasp.right_contact.translation()).norm() >
              closed_chain_contact_position_error_ ||
              poseAngularError(
              interpolated.getGlobalLinkTransform(left_tcp_), expected_grasp.left_contact) >
              closed_chain_contact_orientation_error_ ||
              poseAngularError(
              interpolated.getGlobalLinkTransform(right_tcp_), expected_grasp.right_contact) >
              closed_chain_contact_orientation_error_)
            {
              edge_valid = false;
              break;
            }
          }
          if (!edge_valid) {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "interpolated edge violates collision or rigid contact closure";
            continue;
          }
          ClosedChainSolution solution;
          solution.joints = std::move(joints);
          solution.realized_pose = box_pose;
          solution.joint_margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
          solution.maximum_joint_step = largest_step;
          solution.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
          solutions.push_back(std::move(solution));
          solve_report.failure = ClosedChainFailure::NONE;
        }
        return solutions;
      };
    if (!closed_chain_planner_->search(
        start_joints, controls, deadline, canceled, solve, path, report))
    {
      error = route + " " + report.segment + " waypoint " +
        std::to_string(report.waypoint) + " rejected: " + report.detail;
      publishPlanningDiagnostic(
        report, route, controls.back().pose, controls.front().pose, 0.0, 0.0);
      RCLCPP_WARN(node_->get_logger(), "%s", error.c_str());
      return false;
    }
    nav_msgs::msg::Path box_path;
    box_path.header.frame_id = planning_frame_;
    box_path.header.stamp = node_->now();
    double minimum_margin = std::numeric_limits<double>::infinity();
    double largest_step = 0.0;
    for (const auto & solution : path.states) {
      state.setJointGroupPositions(dual_group, solution.joints);
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = box_path.header;
      pose.pose = toPoseMsg(solution.realized_pose);
      box_path.poses.push_back(std::move(pose));
      minimum_margin = std::min(minimum_margin, solution.joint_margin);
      largest_step = std::max(largest_step, solution.maximum_joint_step);
    }
    box_path_pub_->publish(box_path);
    publishPlanningDiagnostic(
      report, route, controls.back().pose,
      path.states.empty() ? controls.front().pose : path.states.back().realized_pose,
      minimum_margin, largest_step);
    return true;
  }

  bool solveClosedChainWaypoint(
    moveit::core::RobotState & state, const Eigen::Isometry3d & nominal_box_pose,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    const moveit::core::RobotState & previous, bool plan_only, bool allow_projection,
    Eigen::Isometry3d & realized_box_pose, std::string & error)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounds = false;
    bool found_continuous = false;
    for (const auto & box_pose : sharedPoseCandidates(nominal_box_pose, allow_projection)) {
      const auto grasp = graspFromBoxToTcp(
        box_pose, box_to_left_contact, box_to_right_contact, 0.0);
      for (int attempt = 0; attempt < closed_chain_ik_attempts_; ++attempt) {
        moveit::core::RobotState candidate(previous);
        const bool solved = attempt == 0 ?
          setFromDualArmIK(
          candidate, grasp.left_contact, grasp.right_contact, true, closed_chain_ik_timeout_) :
          ([&]() {
            candidate.setToRandomPositions(dual_group);
            return setFromDualArmIK(
              candidate, grasp.left_contact, grasp.right_contact, false,
              closed_chain_ik_timeout_);
          })();
        if (!solved) {
          continue;
        }
        found_ik = true;
        candidate.update();
        if (!candidate.satisfiesBounds(dual_group)) {
          continue;
        }
        found_bounds = true;
        double largest_joint_step = 0.0;
        for (const auto * joint : dual_group->getActiveJointModels()) {
          largest_joint_step = std::max(
            largest_joint_step,
            joint->distance(previous.getJointPositions(joint), candidate.getJointPositions(joint)));
        }
        if (largest_joint_step > max_joint_step_) {
          continue;
        }
        found_continuous = true;
        const bool collision_free = plan_only ?
          collisionFreeWithBox(candidate, box_pose, true) :
          collisionFree(candidate, true, false);
        if (!collision_free) {
          continue;
        }
        state = candidate;
        realized_box_pose = box_pose;
        return true;
      }
    }
    error = !found_ik ? "dual-arm IK failed" :
      !found_bounds ? "dual-arm joint bounds violated" :
      !found_continuous ? "no continuous joint solution" : "state or held box is in collision";
    return false;
  }

  bool solveGraspWaypoint(
    moveit::core::RobotState & state, const GraspGeometry & grasp,
    const moveit::core::RobotState & previous, bool allow_pad_contact, bool ignore_box,
    std::string & error)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    if (!setFromDualArmIK(state, grasp.left_contact, grasp.right_contact, true)) {
      error = "dual-arm IK failed";
      return false;
    }
    if (!state.satisfiesBounds(dual_group)) {
      error = "dual-arm joint bounds violated";
      return false;
    }
    double largest_joint_step = 0.0;
    for (const auto * joint : dual_group->getActiveJointModels()) {
      largest_joint_step = std::max(
        largest_joint_step,
        joint->distance(previous.getJointPositions(joint), state.getJointPositions(joint)));
    }
    if (largest_joint_step > max_joint_step_) {
      error = "largest joint step " + std::to_string(largest_joint_step) +
        " exceeds maximum " + std::to_string(max_joint_step_);
      return false;
    }
    if (!collisionFree(state, allow_pad_contact, ignore_box)) {
      error = "state is in collision";
      return false;
    }
    return true;
  }

  bool appendObjectSegment(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, bool allow_pad_contact,
    bool ignore_box, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, const char * segment,
    std::string & error)
  {
    (void)allow_pad_contact;
    const std::vector<ClosedChainWaypoint> controls{
      {from, false, segment}, {to, true, segment}};
    return appendObjectPath(
      trajectory, state, controls, ignore_box, box_to_left_contact,
      box_to_right_contact, segment,
      std::chrono::steady_clock::now() + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(carry_search_timeout_)),
      []() {return false;}, error);
  }

  bool buildApproach(
    const moveit::core::RobotState & start, const GraspGeometry & target,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    const CancelFunction & canceled)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    std::vector<double> start_joints;
    state.copyJointGroupPositions(dual_group, start_joints);
    Eigen::Isometry3d parameter_start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d parameter_end = Eigen::Isometry3d::Identity();
    parameter_end.translation().x() = pregrasp_distance_;
    const std::vector<ClosedChainWaypoint> controls{
      {parameter_start, false, "approach"}, {parameter_end, false, "approach"}};
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(carry_search_timeout_));
    ClosedChainPath path;
    ClosedChainSearchReport report;
    const moveit::core::RobotState fixed_state(start);
    const ClosedChainSolve solve =
      [this, &fixed_state, dual_group, &target](
        const std::vector<double> & seed, const Eigen::Isometry3d & from_parameter_pose,
        const Eigen::Isometry3d & parameter_pose,
        std::size_t solution_limit, const std::chrono::steady_clock::time_point & solve_deadline,
        const ClosedChainCancel & solve_canceled, ClosedChainSearchReport & solve_report) {
        std::vector<ClosedChainSolution> solutions;
        const double t = pregrasp_distance_ > 1e-9 ?
          std::clamp(parameter_pose.translation().x() / pregrasp_distance_, 0.0, 1.0) : 1.0;
        const Eigen::Isometry3d left = interpolatePose(
          target.left_pregrasp, target.left_contact, t);
        const Eigen::Isometry3d right = interpolatePose(
          target.right_pregrasp, target.right_contact, t);
        for (int attempt = 0; attempt < closed_chain_ik_attempts_ &&
          solutions.size() < solution_limit; ++attempt)
        {
          if (solve_canceled() || std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = solve_canceled() ? ClosedChainFailure::CANCELED :
              ClosedChainFailure::DEADLINE;
            break;
          }
          moveit::core::RobotState candidate(fixed_state);
          candidate.setJointGroupPositions(dual_group, seed);
          candidate.update();
          if (attempt > 0) {
            candidate.setToRandomPositions(dual_group);
          }
          const auto * left_group = candidate.getJointModelGroup(left_group_name_);
          const auto * right_group = candidate.getJointModelGroup(right_group_name_);
          const double remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double left_timeout = std::min(
            closed_chain_ik_timeout_, std::max(1e-4, remaining * 0.5));
          const std::vector<double> left_consistency(
            left_group->getVariableCount(), max_joint_step_);
          const bool left_solved = attempt == 0 ?
            candidate.setFromIK(left_group, left, left_tcp_, left_consistency, left_timeout) :
            candidate.setFromIK(left_group, left, left_tcp_, left_timeout);
          if (!left_solved) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "left arm approach IK failed";
            solve_report.failed_arm = "left";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (solve_canceled() || std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = solve_canceled() ? ClosedChainFailure::CANCELED :
              ClosedChainFailure::DEADLINE;
            break;
          }
          const double right_remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const std::vector<double> right_consistency(
            right_group->getVariableCount(), max_joint_step_);
          const bool right_solved = attempt == 0 ? candidate.setFromIK(
            right_group, right, right_tcp_, right_consistency,
            std::min(closed_chain_ik_timeout_, std::max(1e-4, right_remaining))) :
            candidate.setFromIK(
            right_group, right, right_tcp_,
            std::min(closed_chain_ik_timeout_, std::max(1e-4, right_remaining)));
          if (!right_solved) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "right arm approach IK failed";
            solve_report.failed_arm = "right";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (!candidate.satisfiesBounds(dual_group)) {
            solve_report.failure = ClosedChainFailure::BOUNDS;
            continue;
          }
          moveit::core::RobotState predecessor(fixed_state);
          predecessor.setJointGroupPositions(dual_group, seed);
          predecessor.update();
          double largest_step = 0.0;
          for (const auto * joint : dual_group->getActiveJointModels()) {
            largest_step = std::max(largest_step, joint->distance(
              predecessor.getJointPositions(joint), candidate.getJointPositions(joint)));
          }
          if (largest_step > max_joint_step_) {
            solve_report.failure = ClosedChainFailure::CONTINUITY;
            continue;
          }
          if ((candidate.getGlobalLinkTransform(left_tcp_).translation() -
            left.translation()).norm() > closed_chain_contact_position_error_ ||
            (candidate.getGlobalLinkTransform(right_tcp_).translation() -
            right.translation()).norm() > closed_chain_contact_position_error_ ||
            poseAngularError(candidate.getGlobalLinkTransform(left_tcp_), left) >
            closed_chain_contact_orientation_error_ ||
            poseAngularError(candidate.getGlobalLinkTransform(right_tcp_), right) >
            closed_chain_contact_orientation_error_)
          {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "approach TCP pose closure violated";
            continue;
          }
          if (!collisionFree(candidate, true, false)) {
            solve_report.failure = ClosedChainFailure::COLLISION;
            continue;
          }
          bool edge_valid = true;
          const int edge_samples = std::max(2, static_cast<int>(std::ceil(std::max(
            largest_step / std::max(1e-6, max_joint_step_ * 0.5),
            (parameter_pose.translation() - from_parameter_pose.translation()).norm() /
            closed_chain_validation_position_step_))));
          for (int sample = 1; sample < edge_samples; ++sample) {
            const double edge_t = static_cast<double>(sample) / edge_samples;
            moveit::core::RobotState interpolated(fixed_state);
            predecessor.interpolate(candidate, edge_t, interpolated);
            interpolated.update();
            const double parameter = from_parameter_pose.translation().x() + edge_t *
              (parameter_pose.translation().x() - from_parameter_pose.translation().x());
            const double approach_t = pregrasp_distance_ > 1e-9 ?
              std::clamp(parameter / pregrasp_distance_, 0.0, 1.0) : 1.0;
            const Eigen::Isometry3d expected_left = interpolatePose(
              target.left_pregrasp, target.left_contact, approach_t);
            const Eigen::Isometry3d expected_right = interpolatePose(
              target.right_pregrasp, target.right_contact, approach_t);
            if (!interpolated.satisfiesBounds(dual_group) ||
              !collisionFree(interpolated, true, false) ||
              (interpolated.getGlobalLinkTransform(left_tcp_).translation() -
              expected_left.translation()).norm() > closed_chain_contact_position_error_ ||
              (interpolated.getGlobalLinkTransform(right_tcp_).translation() -
              expected_right.translation()).norm() > closed_chain_contact_position_error_ ||
              poseAngularError(interpolated.getGlobalLinkTransform(left_tcp_), expected_left) >
              closed_chain_contact_orientation_error_ ||
              poseAngularError(interpolated.getGlobalLinkTransform(right_tcp_), expected_right) >
              closed_chain_contact_orientation_error_)
            {
              edge_valid = false;
              break;
            }
          }
          if (!edge_valid) {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "interpolated approach edge violates TCP closure";
            continue;
          }
          ClosedChainSolution solution;
          candidate.copyJointGroupPositions(dual_group, solution.joints);
          solution.realized_pose = parameter_pose;
          solution.joint_margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
          solution.maximum_joint_step = largest_step;
          solutions.push_back(std::move(solution));
          solve_report.failure = ClosedChainFailure::NONE;
        }
        return solutions;
      };
    if (!closed_chain_planner_->search(
        start_joints, controls, deadline, canceled, solve, path, report))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Approach waypoint %zu rejected: %s (classification=%s, failed_arm=%s, "
        "ik_mode=%s, attempts=%zu)",
        report.waypoint, report.detail.c_str(), closedChainFailureName(report.failure),
        report.failed_arm.c_str(), report.ik_mode.c_str(), report.attempts);
      return false;
    }
    for (const auto & solution : path.states) {
      state.setJointGroupPositions(dual_group, solution.joints);
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      return false;
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & place_pose, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += lift_height_;
    Eigen::Isometry3d place_lift = place_pose;
    place_lift.translation().z() += lift_height_;
    const bool ignore_box = plan_only;
    if (!appendObjectSegment(
        trajectory, state, pick_pose, pick_lift, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "pick lift", error) ||
      !appendObjectSegment(
        trajectory, state, pick_lift, place_lift, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "transport", error) ||
      !appendObjectSegment(
        trajectory, state, place_lift, place_pose, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "place descent", error))
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
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += lift_height_;
    if (!appendObjectSegment(
        trajectory, state, pick_pose, pick_lift, false, plan_only,
        box_to_left_contact, box_to_right_contact, "pick lift", error) ||
      !appendObjectSegment(
        trajectory, state, pick_lift, carry_pose, false, plan_only,
        box_to_left_contact, box_to_right_contact, "move to carry", error))
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

  std::vector<Eigen::Isometry3d> carryPoseCandidates(
    const Eigen::Isometry3d & pick_pose) const
  {
    const Eigen::Quaterniond configured(carry_pose_.linear());
    const Eigen::Quaterniond measured(pick_pose.linear());
    const double yaw = std::atan2(pick_pose.linear()(1, 0), pick_pose.linear()(0, 0));
    const Eigen::Quaterniond upright_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const auto bounded_orientation = [this, &configured](const Eigen::Quaterniond & target) {
        const double angle = 2.0 * std::acos(
          std::clamp(std::abs(configured.dot(target)), 0.0, 1.0));
        const double interpolation = angle > 1e-9 ?
          std::min(1.0, carry_search_orientation_tolerance_ / angle) : 0.0;
        return configured.slerp(interpolation, target).normalized();
      };
    const Eigen::Quaterniond toward_upright_yaw = bounded_orientation(upright_yaw);
    const Eigen::Quaterniond toward_measured = bounded_orientation(measured);
    const std::array<Eigen::Quaterniond, 3> orientations{
      configured.normalized(), toward_upright_yaw, toward_measured};
    const std::vector<double> x_offsets{
      0.0, -carry_search_x_range_ / 2.0, carry_search_x_range_ / 2.0,
      -carry_search_x_range_, carry_search_x_range_};
    const std::vector<double> y_offsets{0.0, -carry_search_y_range_, carry_search_y_range_};
    std::vector<double> z_offsets{0.0};
    for (double offset = 0.03; offset <= carry_search_z_lower_ + 1e-9; offset += 0.03) {
      z_offsets.push_back(-offset);
    }
    if (carry_search_z_upper_ > 1e-9) {
      z_offsets.push_back(carry_search_z_upper_);
    }

    struct ScoredPose {double cost; Eigen::Isometry3d pose;};
    std::vector<ScoredPose> scored;
    for (std::size_t orientation_index = 0; orientation_index < orientations.size();
      ++orientation_index)
    {
      for (const double z : z_offsets) {
        for (const double x : x_offsets) {
          for (const double y : y_offsets) {
            Eigen::Isometry3d pose = carry_pose_;
            pose.translation() += Eigen::Vector3d(x, y, z);
            pose.linear() = orientations[orientation_index].toRotationMatrix();
            const double cost = std::abs(x) / std::max(0.001, carry_search_x_range_) +
              std::abs(y) / std::max(0.001, carry_search_y_range_) +
              std::abs(z) / std::max(0.001, carry_search_z_lower_) +
              static_cast<double>(orientation_index) * 0.25;
            scored.push_back({cost, pose});
          }
        }
      }
    }
    std::stable_sort(
      scored.begin(), scored.end(),
      [](const ScoredPose & lhs, const ScoredPose & rhs) {return lhs.cost < rhs.cost;});
    std::vector<Eigen::Isometry3d> result;
    for (const auto & candidate : scored) {
      bool duplicate = false;
      for (const auto & existing : result) {
        const Eigen::Quaterniond qa(existing.linear());
        const Eigen::Quaterniond qb(candidate.pose.linear());
        if ((existing.translation() - candidate.pose.translation()).norm() < 1e-9 &&
          std::abs(qa.dot(qb)) > 1.0 - 1e-9)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        result.push_back(candidate.pose);
      }
      if (result.size() >= static_cast<std::size_t>(maximum_carry_candidates_)) {
        break;
      }
    }
    return result;
  }

  bool buildCarryRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & target_pose, CarryRoute route, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d lift = pick_pose;
    lift.translation().z() += lift_height_;
    std::vector<ClosedChainWaypoint> controls{{pick_pose, false, "pick_lift"}};
    const auto add = [&controls](const Eigen::Isometry3d & pose, const char * segment) {
        controls.push_back({pose, true, segment});
      };
    if (route == CarryRoute::DIRECT) {
      add(lift, "pick_lift");
      add(target_pose, "direct");
    } else if (route == CarryRoute::LOW_XY_THEN_LIFT) {
      Eigen::Isometry3d low = target_pose;
      low.translation().z() = pick_pose.translation().z();
      low.linear() = pick_pose.linear();
      add(low, "low_xy_translation");
      add(target_pose, "lift_after_translation");
    } else if (route == CarryRoute::LIFT_THEN_XY) {
      Eigen::Isometry3d high_target = target_pose;
      high_target.translation().z() = lift.translation().z();
      high_target.linear() = lift.linear();
      add(lift, "pick_lift");
      add(high_target, "high_xy_translation");
      add(target_pose, "carry_descent_rotation");
    } else if (route == CarryRoute::ROTATE_BEFORE_TRANSLATION) {
      Eigen::Isometry3d rotated = lift;
      rotated.linear() = target_pose.linear();
      add(lift, "pick_lift");
      add(rotated, "rotate_before_translation");
      add(target_pose, "carry_translation");
    } else if (route == CarryRoute::ROTATE_AFTER_TRANSLATION) {
      Eigen::Isometry3d translated = target_pose;
      translated.linear() = lift.linear();
      add(lift, "pick_lift");
      add(translated, "carry_translation");
      add(target_pose, "rotate_after_translation");
    } else {
      Eigen::Isometry3d dogleg = interpolatePose(lift, target_pose, 0.5);
      const double sign = route == CarryRoute::DOGLEG_NEGATIVE_Y ? -1.0 : 1.0;
      dogleg.translation().y() += sign * carry_search_y_range_;
      dogleg.translation().x() = std::clamp(
        dogleg.translation().x(), carry_pose_.translation().x() - carry_search_x_range_,
        carry_pose_.translation().x() + carry_search_x_range_);
      add(lift, "pick_lift");
      add(dogleg, "bounded_dogleg");
      add(target_pose, "dogleg_to_carry");
    }
    if (!appendObjectPath(
        trajectory, state, controls, plan_only, box_to_left_contact,
        box_to_right_contact, carryRouteName(route), deadline, canceled, error))
    {
      return false;
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      error = canceled() ? "carry time parameterization canceled" :
        "carry route deadline reached before time parameterization";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      error = "carry time parameterization failed";
      return false;
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      error = canceled() ? "carry time parameterization canceled" :
        "carry route deadline reached during time parameterization";
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool planAdaptiveCarry(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    bool plan_only, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
    std::string & error, const CancelFunction & canceled)
  {
    const std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(carry_search_timeout_));
    const std::array<CarryRoute, 7> routes{
      CarryRoute::DIRECT, CarryRoute::LOW_XY_THEN_LIFT, CarryRoute::LIFT_THEN_XY,
      CarryRoute::ROTATE_BEFORE_TRANSLATION, CarryRoute::ROTATE_AFTER_TRANSLATION,
      CarryRoute::DOGLEG_NEGATIVE_Y, CarryRoute::DOGLEG_POSITIVE_Y};
    struct Endpoint
    {
      Eigen::Isometry3d pose;
      double correction;
      double margin;
      double distance;
      std::size_t order;
    };
    std::vector<Endpoint> endpoints;
    const auto * dual_group = start.getJointModelGroup(move_group_.getName());
    std::size_t endpoint_order = 0;
    const auto precheck_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(carry_search_timeout_ * 0.15));
    for (const auto & pose : carryPoseCandidates(pick_pose)) {
      if (canceled()) {
        error = "adaptive carry endpoint precheck canceled";
        return false;
      }
      if (std::chrono::steady_clock::now() >= precheck_deadline ||
        std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      const auto grasp = graspFromBoxToTcp(
        pose, box_to_left_contact, box_to_right_contact, 0.0);
      moveit::core::RobotState endpoint(start);
      const double remaining = std::chrono::duration<double>(
        precheck_deadline - std::chrono::steady_clock::now()).count();
      if (!setFromDualArmIK(
          endpoint, grasp.left_contact, grasp.right_contact, false,
          std::max(1e-4, std::min(0.02, remaining))))
      {
        ++endpoint_order;
        continue;
      }
      endpoint.update();
      if (!endpoint.satisfiesBounds(dual_group) ||
        !(plan_only ? collisionFreeWithBox(endpoint, pose, true) :
        collisionFree(endpoint, true, false)))
      {
        ++endpoint_order;
        continue;
      }
      endpoints.push_back({
        pose,
        (pose.translation() - carry_pose_.translation()).norm() +
        0.1 * poseAngularError(pose, carry_pose_),
        endpoint.getMinDistanceToPositionBounds(dual_group).first,
        endpoint.distance(start, dual_group), endpoint_order++});
    }
    std::stable_sort(endpoints.begin(), endpoints.end(), [](const Endpoint & a, const Endpoint & b) {
        const double score_a = 4.0 * a.correction + 0.1 * a.distance - 0.2 * a.margin;
        const double score_b = 4.0 * b.correction + 0.1 * b.distance - 0.2 * b.margin;
        return score_a < score_b || (score_a == score_b && a.order < b.order);
      });
    if (endpoints.empty()) {
      error = "no carry endpoint passed IK, bounds, and collision precheck";
      return false;
    }

    std::string last_error = "no carry route evaluated";
    // Sweep all prechecked endpoints for a route before trying a more complex
    // skeleton. This prevents a high but disconnected endpoint from consuming
    // the budget while lower connected poses remain unevaluated.
    for (const auto route : routes) {
      for (const auto & endpoint : endpoints) {
        if (canceled()) {
          error = "adaptive carry search canceled";
          return false;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          error = "adaptive carry search timed out; last failure: " + last_error;
          return false;
        }
        try {
          const std::chrono::steady_clock::time_point route_deadline = std::min(
            deadline, std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(3.25)));
          moveit_msgs::msg::RobotTrajectory trajectory;
          moveit::core::RobotState end(start);
          std::string candidate_error;
          if (!buildCarryRoute(
              start, pick_pose, endpoint.pose, route, plan_only,
              box_to_left_contact, box_to_right_contact,
              trajectory, end, candidate_error, route_deadline, canceled))
          {
            last_error = candidate_error;
            continue;
          }
          selected.pose = endpoint.pose;
          selected.route = route;
          selected.trajectory = std::move(trajectory);
          selected.end_state = std::make_shared<moveit::core::RobotState>(end);
          RCLCPP_INFO(
            node_->get_logger(),
            "Selected adaptive carry pose [%.3f, %.3f, %.3f] using route %s",
            endpoint.pose.translation().x(), endpoint.pose.translation().y(),
            endpoint.pose.translation().z(),
            carryRouteName(route));
          return true;
        } catch (const std::exception & exception) {
          last_error = exception.what();
        }
      }
    }
    error = "no feasible carry pose inside the configured safety envelope; last failure: " +
      last_error;
    return false;
  }

  bool buildPlaceRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & held_pose,
    const Eigen::Isometry3d & place_pose, bool from_pick, CarryRoute route,
    bool ignore_box, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    const auto controls = makePlaceRouteWaypoints(
      held_pose, place_pose, lift_height_, carry_search_y_range_, from_pick, route);

    if (!appendObjectPath(
        trajectory, state, controls, ignore_box, box_to_left_contact,
        box_to_right_contact, std::string("place_") + carryRouteName(route),
        deadline, canceled, error))
    {
      return false;
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      error = canceled() ? "place time parameterization canceled" :
        "place route deadline reached before time parameterization";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, velocity_scaling_, acceleration_scaling_))
    {
      error = "place time parameterization failed";
      return false;
    }
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      error = canceled() ? "place time parameterization canceled" :
        "place route deadline reached during time parameterization";
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  std::vector<Eigen::Isometry3d> placePoseCandidates(
    const Eigen::Isometry3d & requested) const
  {
    const std::array<double, 3> x{0.0, -adaptive_place_position_tolerance_.x(),
      adaptive_place_position_tolerance_.x()};
    const std::array<double, 3> y{0.0, -adaptive_place_position_tolerance_.y(),
      adaptive_place_position_tolerance_.y()};
    const std::array<double, 3> z{0.0, adaptive_place_position_tolerance_.z(),
      -adaptive_place_position_tolerance_.z()};
    const std::array<double, 3> yaw{0.0, -adaptive_place_yaw_tolerance_,
      adaptive_place_yaw_tolerance_};
    struct ScoredPose {double cost; Eigen::Isometry3d pose;};
    std::vector<ScoredPose> scored;
    for (const double dx : x) {
      for (const double dy : y) {
        for (const double dz : z) {
          for (const double angle : yaw) {
            auto pose = requested;
            pose.translation() += Eigen::Vector3d(dx, dy, dz);
            pose.linear() = pose.linear() *
              Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            scored.push_back({
              std::abs(dx) + std::abs(dy) + 2.0 * std::abs(dz) + 0.1 * std::abs(angle), pose});
          }
        }
      }
    }
    std::stable_sort(
      scored.begin(), scored.end(),
      [](const ScoredPose & lhs, const ScoredPose & rhs) {return lhs.cost < rhs.cost;});
    std::vector<Eigen::Isometry3d> result;
    result.reserve(scored.size());
    for (const auto & item : scored) {
      result.push_back(item.pose);
    }
    return result;
  }

  bool planAdaptivePlace(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & requested_pose, bool from_pick, bool ignore_box,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    Eigen::Isometry3d & selected_pose, std::string & error,
    const CancelFunction & canceled)
  {
    const std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(carry_search_timeout_));
    const auto * dual_group = start.getJointModelGroup(move_group_.getName());
    struct Endpoint
    {
      Eigen::Isometry3d pose;
      double correction;
      double margin;
      double distance;
      std::size_t order;
    };
    std::vector<Endpoint> endpoints;
    const auto precheck_deadline = std::min(
      deadline, std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(carry_search_timeout_ * 0.15)));
    std::size_t endpoint_order = 0;
    for (const auto & pose : placePoseCandidates(requested_pose)) {
      if (canceled()) {
        error = "adaptive place endpoint precheck canceled";
        return false;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= precheck_deadline || now >= deadline) {
        break;
      }
      const auto grasp = graspFromBoxToTcp(
        pose, box_to_left_contact, box_to_right_contact, 0.0);
      moveit::core::RobotState endpoint(start);
      const double remaining = std::chrono::duration<double>(precheck_deadline - now).count();
      if (!setFromDualArmIK(
          endpoint, grasp.left_contact, grasp.right_contact, false,
          std::max(1e-4, std::min(0.02, remaining * 0.5))))
      {
        ++endpoint_order;
        continue;
      }
      endpoint.update();
      if (!endpoint.satisfiesBounds(dual_group) ||
        !(ignore_box ? collisionFreeWithBox(endpoint, pose, true) :
        collisionFree(endpoint, true, false)))
      {
        ++endpoint_order;
        continue;
      }
      endpoints.push_back({
        pose,
        (pose.translation() - requested_pose.translation()).norm() +
        0.1 * poseAngularError(pose, requested_pose),
        endpoint.getMinDistanceToPositionBounds(dual_group).first,
        endpoint.distance(start, dual_group), endpoint_order++});
    }
    std::stable_sort(endpoints.begin(), endpoints.end(), [](const Endpoint & a, const Endpoint & b) {
        const double score_a = 4.0 * a.correction + 0.1 * a.distance - 0.2 * a.margin;
        const double score_b = 4.0 * b.correction + 0.1 * b.distance - 0.2 * b.margin;
        return score_a < score_b || (score_a == score_b && a.order < b.order);
      });
    if (endpoints.empty()) {
      error = "no place endpoint passed IK, bounds, and collision precheck";
      return false;
    }

    const std::array<CarryRoute, 7> routes{
      CarryRoute::DIRECT, CarryRoute::LIFT_THEN_XY,
      CarryRoute::ROTATE_BEFORE_TRANSLATION, CarryRoute::ROTATE_AFTER_TRANSLATION,
      CarryRoute::LOW_XY_THEN_LIFT, CarryRoute::DOGLEG_NEGATIVE_Y,
      CarryRoute::DOGLEG_POSITIVE_Y};
    std::string last_error = "no place route evaluated";
    // Try alternate route geometry for the best endpoint before spending the
    // entire budget on small endpoint corrections. A failed route receives a
    // bounded slice, while every solver still observes the common hard deadline.
    for (const auto & endpoint : endpoints) {
      for (const auto route : routes) {
        if (canceled()) {
          error = "adaptive place search canceled";
          return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
          error = "adaptive place search timed out; last failure: " + last_error;
          return false;
        }
        const auto remaining = std::chrono::duration<double>(deadline - now).count();
        const auto route_deadline = std::min(
          deadline, now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::min(1.25, remaining))));
        try {
          moveit_msgs::msg::RobotTrajectory trajectory;
          moveit::core::RobotState candidate_end(start);
          std::string candidate_error;
          if (!buildPlaceRoute(
              start, from_pose, endpoint.pose, from_pick, route, ignore_box,
              box_to_left_contact, box_to_right_contact, trajectory, candidate_end,
              candidate_error, route_deadline, canceled))
          {
            last_error = candidate_error;
            continue;
          }
          output = std::move(trajectory);
          end_state = candidate_end;
          selected_pose = endpoint.pose;
          RCLCPP_INFO(
            node_->get_logger(),
            "Selected adaptive place offset [%.3f, %.3f, %.3f] m using route %s",
            endpoint.pose.translation().x() - requested_pose.translation().x(),
            endpoint.pose.translation().y() - requested_pose.translation().y(),
            endpoint.pose.translation().z() - requested_pose.translation().z(),
            carryRouteName(route));
          return true;
        } catch (const std::exception & exception) {
          last_error = exception.what();
        }
      }
    }
    error = "no feasible place pose inside the configured tolerance; last failure: " + last_error;
    return false;
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

  bool setValidatedPlanningTarget(
    moveit::core::RobotState & target, double & maximum_normalization,
    bool & move_group_reported_bounds, std::string & error)
  {
    const auto * group = target.getJointModelGroup(move_group_.getName());
    if (!group) {
      error = "planning group is unavailable while assigning the joint target";
      return false;
    }
    std::vector<double> original;
    target.copyJointGroupPositions(group, original);
    target.enforceBounds(group);
    target.update();
    std::vector<double> normalized;
    target.copyJointGroupPositions(group, normalized);
    if (normalized.size() != group->getVariableCount() || normalized.size() != original.size()) {
      error = "planning target has the wrong number of group variables";
      return false;
    }
    maximum_normalization = 0.0;
    for (std::size_t index = 0; index < normalized.size(); ++index) {
      if (!std::isfinite(normalized[index])) {
        error = "planning target contains a non-finite joint value";
        return false;
      }
      maximum_normalization = std::max(
        maximum_normalization, std::abs(normalized[index] - original[index]));
    }
    if (!target.satisfiesBounds(group, 1e-9)) {
      const auto [distance, joint] = target.getMinDistanceToPositionBounds(group);
      error = "normalized planning target violates bounds";
      if (joint) {
        error += " at " + joint->getName() + " (margin=" + std::to_string(distance) + ")";
      }
      return false;
    }
    if (!collisionFree(target, false, false)) {
      error = "normalized planning target is in collision";
      return false;
    }

    move_group_reported_bounds = !move_group_.setJointValueTarget(normalized);
    std::vector<double> installed;
    move_group_.getJointValueTarget(installed);
    if (installed.size() != normalized.size()) {
      error = "MoveGroup stored the wrong number of target variables";
      return false;
    }
    for (std::size_t index = 0; index < installed.size(); ++index) {
      if (!std::isfinite(installed[index]) ||
        std::abs(installed[index] - normalized[index]) > 1e-9)
      {
        error = "MoveGroup did not store the validated planning-group target";
        return false;
      }
    }
    return true;
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

  void clearGraspMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
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
    PlannedGrasp & selected, const ContinuationFunction & continuation, std::string & error,
    const CancelFunction & canceled)
  {
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable";
      return false;
    }
    current->update();
    GraspCandidateOptions options;
    options.position_tolerance = grasp_position_tolerance_;
    options.orientation_tolerance = grasp_orientation_tolerance_;
    options.pregrasp_distance_tolerance = pregrasp_distance_tolerance_;
    options.alternate_face_alignment_tolerance = alternate_face_alignment_tolerance_;
    options.maximum_candidates = static_cast<std::size_t>(maximum_grasp_candidates_);
    const auto candidates = generateGraspCandidates(
      pick_pose, dimensions_, pregrasp_distance_, contact_height_offset_, options);

    struct FeasibleCandidate
    {
      PlannedGrasp grasp;
      moveit::core::RobotState state;
    };
    std::vector<FeasibleCandidate> feasible;
    int ik_rejected = 0;
    int bounds_rejected = 0;
    int collision_rejected = 0;
    int margin_rejected = 0;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(grasp_search_timeout_);
    for (const auto & candidate : candidates) {
      if (canceled()) {
        error = "coordinated grasp search canceled";
        return false;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      moveit::core::RobotState pregrasp_goal(*current);
      double margin = 0.0;
      double distance = 0.0;
      std::string candidate_error;
      if (!solvePregraspIK(
          *current, candidate.grasp, ik_attempts_per_candidate_, pregrasp_goal,
          margin, distance, candidate_error))
      {
        if (candidate_error.find("bounds") != std::string::npos) {
          ++bounds_rejected;
        } else if (candidate_error.find("collision") != std::string::npos) {
          ++collision_rejected;
        } else {
          ++ik_rejected;
        }
        continue;
      }
      if (margin < minimum_grasp_joint_margin_) {
        ++margin_rejected;
        continue;
      }
      feasible.push_back({PlannedGrasp{candidate, margin, distance}, pregrasp_goal});
    }
    if (feasible.empty()) {
      error = "no feasible coordinated pregrasp candidate (IK=" +
        std::to_string(ik_rejected) + ", bounds=" + std::to_string(bounds_rejected) +
        ", collision=" + std::to_string(collision_rejected) + ", joint_margin=" +
        std::to_string(margin_rejected) + ")";
      return false;
    }
    std::stable_sort(
      feasible.begin(), feasible.end(),
      [](const FeasibleCandidate & lhs, const FeasibleCandidate & rhs) {
        if (std::abs(
            lhs.grasp.candidate.correction_cost - rhs.grasp.candidate.correction_cost) > 1e-12)
        {
          return lhs.grasp.candidate.correction_cost < rhs.grasp.candidate.correction_cost;
        }
        if (std::abs(lhs.grasp.joint_limit_margin - rhs.grasp.joint_limit_margin) > 1e-12) {
          return lhs.grasp.joint_limit_margin > rhs.grasp.joint_limit_margin;
        }
        return lhs.grasp.joint_distance < rhs.grasp.joint_distance;
      });

    const std::size_t planning_count = std::min(
      feasible.size(), static_cast<std::size_t>(maximum_planning_candidates_));
    const auto planning_started = std::chrono::steady_clock::now();
    const auto planning_elapsed = [&planning_started]() {
        return std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started).count();
      };
    std::vector<std::size_t> retry_candidates;
    std::map<int32_t, int> motion_error_codes;
    int initial_motion_rejected = 0;
    int retry_motion_rejected = 0;
    int target_rejected = 0;
    int approach_rejected = 0;
    int continuation_rejected = 0;
    std::string last_continuation_error;
    enum class CandidateAttempt {
      SUCCESS, TARGET_REJECTED, OMPL_REJECTED, APPROACH_REJECTED, CONTINUATION_REJECTED
    };
    const auto attempt_candidate =
      [this, &current, &feasible, &box_message, &selected, &pregrasp_plan,
        &approach, &contact_end, &motion_error_codes, &target_rejected,
        &approach_rejected, &continuation_rejected,
        &last_continuation_error,
        &continuation, &canceled](std::size_t index, double timeout, const char * phase) {
        active_grasp_id_ = "candidate_" + std::to_string(index + 1U);
        publishGraspMarkers(box_message, feasible[index].grasp.candidate.grasp);
        RCLCPP_INFO(
          node_->get_logger(),
          "Planning pregrasp candidate %zu in %s phase with %.2f s timeout "
          "(correction=%.3f, margin=%.3f, distance=%.3f)",
          index + 1U, phase, timeout, feasible[index].grasp.candidate.correction_cost,
          feasible[index].grasp.joint_limit_margin, feasible[index].grasp.joint_distance);

        move_group_.setStartState(*current);
        move_group_.clearPoseTargets();
        double maximum_normalization = 0.0;
        bool move_group_reported_bounds = false;
        std::string target_error;
        if (!setValidatedPlanningTarget(
            feasible[index].state, maximum_normalization,
            move_group_reported_bounds, target_error))
        {
          ++target_rejected;
          RCLCPP_WARN(
            node_->get_logger(), "Candidate %zu joint target rejected: %s",
            index + 1U, target_error.c_str());
          return CandidateAttempt::TARGET_REJECTED;
        }
        if (maximum_normalization > 1e-9) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Candidate %zu normalized continuous/bounded joints by at most %.6f rad",
            index + 1U, maximum_normalization);
        }
        if (move_group_reported_bounds) {
          RCLCPP_WARN(
            node_->get_logger(),
            "MoveGroup reported candidate %zu out of bounds, but its normalized 14-joint "
            "target passed model bounds/collision checks and was stored exactly; submitting "
            "it to the planning pipeline for authoritative validation",
            index + 1U);
        }
        moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
        move_group_.setPlanningTime(timeout);
        const auto attempt_started = std::chrono::steady_clock::now();
        const auto plan_result = move_group_.plan(candidate_plan);
        const double attempt_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - attempt_started).count();
        move_group_.setPlanningTime(10.0);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
          ++motion_error_codes[plan_result.val];
          RCLCPP_WARN(
            node_->get_logger(),
            "Candidate %zu %s OMPL attempt failed after %.2f s: %s (%d)",
            index + 1U, phase, attempt_elapsed,
            moveit::core::error_code_to_string(plan_result).c_str(), plan_result.val);
          return CandidateAttempt::OMPL_REJECTED;
        }
        auto pregrasp_end = stateAtEnd(*current, candidate_plan.trajectory_);
        moveit_msgs::msg::RobotTrajectory candidate_approach;
        auto candidate_contact_end = pregrasp_end;
        if (!buildApproach(
            pregrasp_end, feasible[index].grasp.candidate.grasp,
            candidate_approach, candidate_contact_end, canceled))
        {
          ++approach_rejected;
          RCLCPP_WARN(
            node_->get_logger(),
            "Candidate %zu reached pregrasp but its Cartesian approach was rejected", index + 1U);
          return CandidateAttempt::APPROACH_REJECTED;
        }
        std::string continuation_error;
        if (continuation &&
          !continuation(candidate_contact_end, feasible[index].grasp, continuation_error))
        {
          ++continuation_rejected;
          last_continuation_error = continuation_error;
          RCLCPP_WARN(
            node_->get_logger(), "Candidate %zu rejected by closed-chain continuation: %s",
            index + 1U, continuation_error.c_str());
          return CandidateAttempt::CONTINUATION_REJECTED;
        }
        pregrasp_plan = std::move(candidate_plan);
        approach = std::move(candidate_approach);
        contact_end = std::move(candidate_contact_end);
        selected = feasible[index].grasp;
        return CandidateAttempt::SUCCESS;
      };

    const auto report_success = [this, &selected, &box_message]() {
        publishGraspMarkers(box_message, selected.candidate.grasp);
        RCLCPP_INFO(
          node_->get_logger(),
          "Selected grasp candidate: correction=%.3f tilt=%.1f deg height=%.3f "
          "tangent=%.3f wrist=%.1f deg clearance=%.3f margin=%.3f distance=%.3f",
          selected.candidate.correction_cost,
          selected.candidate.tilt_correction * 57.29577951308232,
          selected.candidate.contact_height_offset, selected.candidate.tangent_offset,
          selected.candidate.wrist_rotation * 57.29577951308232,
          selected.candidate.pregrasp_distance, selected.joint_limit_margin,
          selected.joint_distance);
      };

    for (std::size_t index = 0; index < planning_count; ++index) {
      if (canceled()) {
        error = "pregrasp planning canceled";
        return false;
      }
      const double remaining = pregrasp_planning_timeout_ - planning_elapsed();
      if (remaining <= 0.0) {
        break;
      }
      const auto result = attempt_candidate(
        index, std::min(planning_time_per_candidate_, remaining), "initial");
      if (canceled()) {
        error = "pregrasp planning canceled";
        return false;
      }
      if (result == CandidateAttempt::SUCCESS) {
        report_success();
        return true;
      }
      if (result == CandidateAttempt::OMPL_REJECTED) {
        ++initial_motion_rejected;
        retry_candidates.push_back(index);
      }
    }

    const std::size_t retry_count = std::min(
      retry_candidates.size(), static_cast<std::size_t>(maximum_retry_candidates_));
    for (std::size_t retry = 0; retry < retry_count; ++retry) {
      if (canceled()) {
        error = "pregrasp retry planning canceled";
        return false;
      }
      const double timeout = adaptiveRetryTimeout(
        pregrasp_planning_timeout_, planning_elapsed(), retry_count - retry);
      if (timeout <= 0.0) {
        break;
      }
      const auto result = attempt_candidate(
        retry_candidates[retry], timeout, "adaptive-retry");
      if (canceled()) {
        error = "pregrasp retry planning canceled";
        return false;
      }
      if (result == CandidateAttempt::SUCCESS) {
        report_success();
        return true;
      }
      if (result == CandidateAttempt::OMPL_REJECTED) {
        ++retry_motion_rejected;
      }
    }

    const double elapsed = planning_elapsed();
    std::ostringstream details;
    bool first_code = true;
    for (const auto & [code, count] : motion_error_codes) {
      if (!first_code) {
        details << ",";
      }
      first_code = false;
      details << moveit::core::error_code_to_string(moveit::core::MoveItErrorCode(code)) <<
        ":" << count;
    }
    std::ostringstream message;
    message << std::fixed << std::setprecision(1) <<
      "coordinated pregrasp candidates failed motion planning after " << elapsed <<
      " s (budget=" << pregrasp_planning_timeout_ << " s, initial_OMPL=" <<
      initial_motion_rejected << ", retry_OMPL=" << retry_motion_rejected <<
      ", target=" << target_rejected << ", approach=" << approach_rejected;
    message << ", continuation=" << continuation_rejected;
    if (!motion_error_codes.empty()) {
      message << ", errors=" << details.str();
    }
    message << ")";
    if (!last_continuation_error.empty()) {
      message << "; last continuation failure: " << last_continuation_error;
    }
    error = message.str();
    return false;
  }

  void updateHeldPoseFromRobot()
  {
    auto current = move_group_.getCurrentState(1.0);
    if (!current) {
      return;
    }
    current->update();
    const Eigen::Isometry3d estimated =
      current->getGlobalLinkTransform(left_tcp_) * held_box_to_left_contact_.inverse();
    held_pose_ = stampedPose(estimated);
  }

  bool validateHeldClosure(std::string & error)
  {
    if (!held_geometry_valid_) {
      error = "held-object contact geometry is unavailable";
      return false;
    }
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable for held-object closure validation";
      return false;
    }
    current->update();
    const auto * dual_group = current->getJointModelGroup(move_group_.getName());
    if (!current->satisfiesBounds(dual_group)) {
      error = "current robot state violates bounds before Place";
      return false;
    }
    const Eigen::Isometry3d left_estimate =
      current->getGlobalLinkTransform(left_tcp_) * held_box_to_left_contact_.inverse();
    const Eigen::Isometry3d right_estimate =
      current->getGlobalLinkTransform(right_tcp_) * held_box_to_right_contact_.inverse();
    const double position_error =
      (left_estimate.translation() - right_estimate.translation()).norm();
    const double orientation_error = poseAngularError(left_estimate, right_estimate);
    if (position_error > closed_chain_contact_position_error_ ||
      orientation_error > closed_chain_contact_orientation_error_)
    {
      error = "left/right TCPs disagree on held box pose (position_error=" +
        std::to_string(position_error) + ", orientation_error=" +
        std::to_string(orientation_error) + ")";
      return false;
    }
    if (!collisionFree(*current, true, false)) {
      error = "current held-object state is in collision before Place";
      return false;
    }
    Eigen::Isometry3d estimated = left_estimate;
    estimated.translation() =
      0.5 * (left_estimate.translation() + right_estimate.translation());
    Eigen::Quaterniond left_rotation(left_estimate.linear());
    Eigen::Quaterniond right_rotation(right_estimate.linear());
    if (left_rotation.dot(right_rotation) < 0.0) {
      right_rotation.coeffs() *= -1.0;
    }
    estimated.linear() =
      left_rotation.slerp(0.5, right_rotation).normalized().toRotationMatrix();
    held_pose_ = stampedPose(estimated);
    return true;
  }

  TaskOutcome runPick(
    bool plan_only, const FeedbackFunction & feedback, const CancelFunction & canceled,
    const geometry_msgs::msg::PoseStamped * requested_place = nullptr)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "Pick requires manipulation state EMPTY");
    }
    geometry_msgs::msg::PoseStamped box_message;
    if (!stableBoxPose(box_message)) {
      return outcome(false, kNoStableBoxPose, "no fresh stable box pose");
    }
    Eigen::Isometry3d pick_pose;
    Eigen::Isometry3d pick_place_target = Eigen::Isometry3d::Identity();
    try {
      pick_pose = toEigen(box_message.pose);
      if (requested_place) {
        geometry_msgs::msg::PoseStamped transformed_place;
        std::string transform_error;
        if (!transformGoalPose(*requested_place, transformed_place, transform_error)) {
          return outcome(false, kInvalidGoal, transform_error);
        }
        pick_place_target = toEigen(transformed_place.pose);
      }
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }

    std::string error;
    if (!applyBox(pick_pose, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before perception refresh");
    }
    if (!refreshOctomap(error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before planning");
    }
    feedback("planning_pregrasp", 0.15F, box_message);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    AdaptiveCarryPlan carry_plan;
    PlannedGrasp selected_grasp;
    const ContinuationFunction carry_validator =
      [this, &pick_pose, &pick_place_target, requested_place, &carry_plan, &canceled](
        const moveit::core::RobotState & candidate_contact,
        const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!planAdaptiveCarry(
            candidate_contact, pick_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
        {
          return false;
        }
        if (!requested_place) {
          return true;
        }
        moveit_msgs::msg::RobotTrajectory place_validation;
        moveit::core::RobotState place_end(*carry_plan.end_state);
        Eigen::Isometry3d selected_place_pose = pick_place_target;
        return planAdaptivePlace(
          *carry_plan.end_state, carry_plan.pose, pick_place_target, false, true,
          candidate.candidate.box_to_left_contact,
          candidate.candidate.box_to_right_contact,
          place_validation, place_end, selected_place_pose, continuation_error, canceled);
      };
    if (!planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, carry_validator, error, canceled))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled after full-path planning");
    }
    if (plan_only) {
      return outcome(
        true, kSuccess, "pick path to adaptive carry pose is feasible",
        stampedPose(carry_plan.pose));
    }

    feedback("executing_pregrasp", 0.30F, box_message);
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before pregrasp execution");
    }
    geometry_msgs::msg::PoseStamped revalidated_box;
    if (!boxPoseStillWithinTolerance(pick_pose, revalidated_box, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (!executeMotion(pregrasp_plan, canceled)) {
      return outcome(false, kExecutionFailed, executionError("pregrasp execution failed"));
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before approach");
    }
    if (!boxPoseStillWithinTolerance(pick_pose, revalidated_box, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    const auto & pick_grasp = selected_grasp.candidate.grasp;
    auto current = move_group_.getCurrentState(2.0);
    if (!current || !buildApproach(*current, pick_grasp, approach, contact_end, canceled)) {
      return outcome(false, kSafetyAbort, "approach revalidation failed");
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before approach execution");
    }
    feedback("approaching", 0.45F, box_message);
    if (!executeMotion(approach, canceled)) {
      return outcome(false, kExecutionFailed, executionError("approach execution failed"));
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before attachment");
    }

    feedback("attaching", 0.60F, box_message);
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before physical attachment");
    }
    bool attach_dispatched = false;
    if (!callAttachmentService(attach_client_, error, &attach_dispatched)) {
      if (attach_dispatched) {
        physical_attachment_expected_.store(true);
        held_pose_ = box_message;
        held_box_to_left_contact_ = selected_grasp.candidate.box_to_left_contact;
        held_box_to_right_contact_ = selected_grasp.candidate.box_to_right_contact;
        held_geometry_valid_ = true;
        setState(
          ManipulationState::RECOVERY_REQUIRED,
          "attachment request was dispatched but its result is uncertain");
        return outcome(
          false, kRecoveryRequired,
          error + "; attachment may exist; explicit recovery/reset is required", held_pose_);
      }
      return outcome(false, kAttachmentFailed, error);
    }
    physical_attachment_expected_.store(simulate_attachment_);
    held_pose_ = box_message;
    held_box_to_left_contact_ = selected_grasp.candidate.box_to_left_contact;
    held_box_to_right_contact_ = selected_grasp.candidate.box_to_right_contact;
    held_geometry_valid_ = true;
    if (canceled()) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "pick canceled after physical attachment but before planning-scene attachment");
      return outcome(
        false, kRecoveryRequired,
        "pick canceled after physical attachment; object may remain held", held_pose_);
    }
    if (!attachBoxToScene(error)) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "physical attachment may exist but MoveIt attachment failed");
      return outcome(
        false, kRecoveryRequired, error + "; physical object may remain held", held_pose_);
    }
    setState(ManipulationState::HOLDING, "box attached at pick pose");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after attachment");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }

    current = move_group_.getCurrentState(2.0);
    const auto carry_revalidation_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(carry_search_timeout_));
    if (!current || !buildCarryRoute(
        *current, pick_pose, carry_plan.pose, carry_plan.route, false,
        held_box_to_left_contact_, held_box_to_right_contact_,
        carry_plan.trajectory, *carry_plan.end_state, error,
        carry_revalidation_deadline, canceled))
    {
      setState(ManipulationState::RECOVERY_REQUIRED, "carry revalidation failed");
      updateHeldPoseFromRobot();
      return outcome(false, kRecoveryRequired, "carry revalidation failed; object remains held", held_pose_);
    }
    feedback("moving_to_carry", 0.80F, box_message);
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled before carry execution");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }
    if (!executeMotion(carry_plan.trajectory, canceled)) {
      updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "carry execution failed");
      return outcome(
        false, kRecoveryRequired,
        executionError("carry execution failed") + "; object remains held", held_pose_);
    }
    if (canceled()) {
      updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after carry execution");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }
    held_pose_ = stampedPose(carry_plan.pose);
    setState(ManipulationState::HOLDING, "box held at carry pose");
    feedback("holding", 1.0F, held_pose_);
    return outcome(true, kSuccess, "box picked and moved to carry pose", held_pose_);
  }

  TaskOutcome runPlace(
    const geometry_msgs::msg::PoseStamped & requested_pose, bool plan_only,
    const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before validation", held_pose_);
    }
    const uint8_t current_state = state_.load();
    if (current_state != ManipulationState::HOLDING) {
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
    if (!validateHeldClosure(error)) {
      setState(ManipulationState::RECOVERY_REQUIRED, error);
      return outcome(false, kRecoveryRequired, error, held_pose_);
    }
    Eigen::Isometry3d from_pose;
    try {
      from_pose = toEigen(held_pose_.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kRecoveryRequired, exception.what(), held_pose_);
    }
    if (!refreshOctomap(error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before planning", held_pose_);
    }
    feedback("planning_place", 0.15F, held_pose_);
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable", held_pose_);
    }
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(*current);
    Eigen::Isometry3d selected_place_pose = place_pose;
    if (!planAdaptivePlace(
        *current, from_pose, place_pose, false, false, held_box_to_left_contact_,
        held_box_to_right_contact_, transport, place_end, selected_place_pose, error, canceled))
    {
      return outcome(false, kPlanningFailed, "adaptive closed-chain place planning failed: " + error,
        held_pose_);
    }
    place_pose = selected_place_pose;
    place_message = stampedPose(place_pose);
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled after planning", held_pose_);
    }
    if (plan_only) {
      return outcome(true, kSuccess, "place path is feasible", place_message);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "place canceled before motion", held_pose_);
    }
    feedback("moving_to_place", 0.45F, held_pose_);
    if (!executeMotion(transport, canceled)) {
      updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "place motion failed");
      return outcome(
        false, kRecoveryRequired,
        executionError("place motion failed") + "; object remains held", held_pose_);
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
    physical_attachment_expected_.store(false);
    if (!placeBoxInScene(place_pose, error)) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "box release completed but planning-scene transition failed");
      return outcome(
        false, kRecoveryRequired,
        error + "; box release completed; operator recovery required", place_message);
    }
    setState(ManipulationState::EMPTY, "box placed; retreat in progress");
    if (canceled()) {
      held_pose_ = geometry_msgs::msg::PoseStamped();
      return outcome(
        false, kExecutionFailed, "box placed, but retreat was canceled", place_message);
    }

    const auto place_grasp = graspFromBoxToTcp(
      place_pose, held_box_to_left_contact_, held_box_to_right_contact_, pregrasp_distance_);
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
    if (!buildApproach(*current, retreat_target, retreat, retreat_end, canceled)) {
      return outcome(false, kExecutionFailed, "box placed, but retreat planning failed", place_message);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "box placed, but retreat was canceled", place_message);
    }
    if (!executeMotion(retreat, canceled)) {
      return outcome(
        false, kExecutionFailed, executionError("box placed, but retreat failed"), place_message);
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
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; return to zero canceled after planning");
      return outcome(
        false, kExecutionFailed, "box placed, but return to zero was canceled", place_message);
    }
    if (!executeMotion(zero_plan, canceled)) {
      setState(ManipulationState::EMPTY, "box placed; return-to-zero execution failed");
      return outcome(
        false, kExecutionFailed,
        executionError("box placed, but return-to-zero execution failed"), place_message);
    }
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; reset requested after return to zero");
      return outcome(
        false, kExecutionFailed, "box placed and arms reached zero, but the action was canceled",
        place_message);
    }
    setState(ManipulationState::EMPTY, "ready to pick; arms at zero");
    feedback("complete", 1.0F, place_message);
    return outcome(true, kSuccess, "box placed and arms returned to zero", place_message);
  }

  TaskOutcome planCompletePath(
    const geometry_msgs::msg::PoseStamped & requested_place, const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before validation");
    }
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
    if (!applyBox(pick_pose, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before perception refresh");
    }
    if (!refreshOctomap(error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before motion planning");
    }
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(move_group_.getRobotModel());
    Eigen::Isometry3d selected_place_pose = place_pose;
    PlannedGrasp selected_grasp;
    AdaptiveCarryPlan carry_plan;
    const ContinuationFunction transport_validator =
      [this, &pick_pose, &place_pose, &transport, &place_end, &selected_place_pose,
        &carry_plan, &canceled](
        const moveit::core::RobotState & candidate_contact,
        const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!planAdaptiveCarry(
            candidate_contact, pick_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
        {
          return false;
        }
        place_end = *carry_plan.end_state;
        return planAdaptivePlace(
          *carry_plan.end_state, carry_plan.pose, place_pose, false, true,
          candidate.candidate.box_to_left_contact,
          candidate.candidate.box_to_right_contact,
          transport, place_end, selected_place_pose, continuation_error, canceled);
      };
    if (!planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, transport_validator, error, canceled))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled after pregrasp planning");
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled after transport planning");
    }
    return outcome(
      true, kSuccess, "complete pick/place path is feasible with adaptive place tolerance",
      stampedPose(selected_place_pose));
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

  void executeReset(const std::shared_ptr<ResetGoalHandle> & goal)
  {
    const auto feedback = [goal](const std::string & stage, float progress) {
        auto message = std::make_shared<ResetManipulation::Feedback>();
        message->stage = stage;
        message->progress = progress;
        goal->publish_feedback(message);
      };
    const auto finish = [this, goal](
      bool success, uint16_t code, const std::string & message, bool clear_reset_latch) {
        auto result = std::make_shared<ResetManipulation::Result>();
        result->success = success;
        result->error_code = code;
        result->message = message;
        const bool canceling = goal->is_canceling();
        if (canceling) {
          result->success = false;
          result->error_code = ResetManipulation::Result::CANCELED;
          result->message = "reset canceled; manipulation remains locked for recovery";
          setState(ManipulationState::RECOVERY_REQUIRED, result->message);
          clear_reset_latch = false;
        }
        if (clear_reset_latch) {
          reset_physical_detach_done_ = false;
          reset_scene_cleanup_done_ = false;
        }
        reset_coordinator_.finishReset(clear_reset_latch);
        if (canceling) {
          goal->canceled(result);
        } else if (success) {
          goal->succeed(result);
        } else {
          goal->abort(result);
        }
      };
    const auto fail = [this, &finish](uint16_t code, const std::string & message) {
        setState(ManipulationState::RECOVERY_REQUIRED, message);
        finish(false, code, message, false);
      };

    setState(
      ManipulationState::RECOVERY_REQUIRED,
      "reset in progress; manipulation remains locked until verification succeeds");
    feedback("preempting", 0.05F);
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(reset_preemption_timeout_));
    const auto access = reset_coordinator_.waitForResetAccess(
      timeout, [goal]() {return goal->is_canceling();});
    if (access == ResetAcquireResult::CANCELED) {
      fail(
        ResetManipulation::Result::CANCELED,
        "reset canceled while waiting for active manipulation to stop");
      return;
    }
    if (access != ResetAcquireResult::ACQUIRED) {
      fail(
        ResetManipulation::Result::PREEMPTION_TIMEOUT,
        "active manipulation did not stop before the reset timeout");
      return;
    }

    try {
      std::string error;
      feedback("clearing_scene", 0.20F);
      if (!reset_physical_detach_done_) {
        if (simulate_attachment_ && physical_attachment_expected_.load() &&
          !callAttachmentService(detach_client_, error))
        {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
        physical_attachment_expected_.store(false);
        reset_physical_detach_done_ = true;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled during physical cleanup");
        return;
      }
      if (!reset_scene_cleanup_done_) {
        if (!clearBoxScene(error)) {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
        reset_scene_cleanup_done_ = true;
      }
      held_pose_ = geometry_msgs::msg::PoseStamped();
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        have_box_pose_ = false;
      }
      clearGraspMarkers();
      move_group_.clearPoseTargets();

      feedback("refreshing_octomap", 0.35F);
      if (perception_source_ == Perception3dSource::NONE) {
        if (!clearOctomap(error) || !synchronizePlanningScene(error)) {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
      } else if (!refreshOctomap(error)) {
        fail(ResetManipulation::Result::CLEANUP_FAILED, error);
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled before zero planning");
        return;
      }

      feedback("planning_zero", 0.50F);
      auto current = move_group_.getCurrentState(reset_state_timeout_);
      if (!current) {
        fail(ResetManipulation::Result::STATE_UNAVAILABLE, "current robot state unavailable");
        return;
      }
      current->update();
      move_group_.setStartState(*current);
      move_group_.clearPoseTargets();
      if (!move_group_.setNamedTarget(reset_named_target_)) {
        fail(
          ResetManipulation::Result::PLANNING_FAILED,
          "configured reset named target is unavailable");
        return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
      if (move_group_.plan(reset_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        fail(
          ResetManipulation::Result::PLANNING_FAILED,
          "collision-checked planning to the reset target failed");
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled before zero execution");
        return;
      }

      feedback("executing_zero", 0.70F);
      execution_cancel_requested_.store(false);
      std::map<std::string, double> settled_reset_positions;
      if (!executeMotion(
          reset_plan, [goal]() {return goal->is_canceling();}, &settled_reset_positions))
      {
        const std::string message = goal->is_canceling() ?
          "reset canceled during zero execution" :
          executionError("execution to the reset target failed");
        fail(
          goal->is_canceling() ? ResetManipulation::Result::CANCELED :
          ResetManipulation::Result::EXECUTION_FAILED,
          message);
        return;
      }

      feedback("verifying", 0.90F);
      const auto verification = verifyJointTarget(
        reset_target_values_, settled_reset_positions, reset_joint_tolerance_);
      if (!verification.within_tolerance) {
        fail(
          ResetManipulation::Result::VERIFICATION_FAILED,
          "reset verification failed at " + verification.joint_name + " (error " +
          std::to_string(verification.error) + " rad)");
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled during verification");
        return;
      }

      held_pose_ = geometry_msgs::msg::PoseStamped();
      setState(ManipulationState::EMPTY, "reset complete; arms at zero", true, true);
      feedback("complete", 1.0F);
      finish(
        true, ResetManipulation::Result::SUCCESS,
        "manipulation state cleared and arms returned to zero", true);
    } catch (const std::exception & exception) {
      fail(
        ResetManipulation::Result::CLEANUP_FAILED,
        "reset failed with exception: " + std::string(exception.what()));
    }
  }

  void executePick(const std::shared_ptr<PickGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Pick::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    TaskOutcome task;
    try {
      task = runPick(
        goal->get_goal()->plan_only, feedback,
        [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();});
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (goal->get_goal()->plan_only) {
        setState(ManipulationState::EMPTY, "plan-only Pick stopped after an exception");
      } else {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected Pick exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "Pick failed with exception: " + std::string(exception.what()));
    }
    release.run();
    finishGoal(goal, task, std::make_shared<Pick::Result>());
  }

  void executePlace(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Place::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    TaskOutcome task;
    try {
      task = runPlace(
        goal->get_goal()->place_pose, goal->get_goal()->plan_only, feedback,
        [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();});
    } catch (const std::exception & exception) {
      move_group_.stop();
      setState(ManipulationState::RECOVERY_REQUIRED, "unexpected Place exception");
      task = outcome(
        false, kRecoveryRequired, "Place failed with exception: " + std::string(exception.what()),
        held_pose_);
    }
    release.run();
    finishGoal(goal, task, std::make_shared<Place::Result>());
  }

  void executePickPlace(const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    TaskOutcome task;
    const CancelFunction canceled =
      [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();};
    try {
      if (goal->get_goal()->plan_only) {
        task = planCompletePath(goal->get_goal()->place_pose, canceled);
      } else {
        const FeedbackFunction pick_feedback = [goal](
          const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
            auto message = std::make_shared<PickPlace::Feedback>();
            message->stage = "pick/" + stage;
            message->progress = progress * 0.5F;
            message->box_pose = pose;
            goal->publish_feedback(message);
          };
        task = runPick(false, pick_feedback, canceled, &goal->get_goal()->place_pose);
        if (task.success) {
          const FeedbackFunction place_feedback = [goal](
            const std::string & stage, float progress,
            const geometry_msgs::msg::PoseStamped & pose) {
              auto message = std::make_shared<PickPlace::Feedback>();
              message->stage = "place/" + stage;
              message->progress = 0.5F + progress * 0.5F;
              message->box_pose = pose;
              goal->publish_feedback(message);
            };
          task = runPlace(goal->get_goal()->place_pose, false, place_feedback, canceled);
        }
        if (!task.success && task.object_held) {
          task.message += "; object remains held";
        }
      }
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (goal->get_goal()->plan_only) {
        setState(ManipulationState::EMPTY, "plan-only PickPlace stopped after an exception");
      } else {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected PickPlace exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "PickPlace failed with exception: " + std::string(exception.what()), held_pose_);
    }
    release.run();
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
  std::string active_grasp_id_{"unassigned"};
  BoxDimensions dimensions_;
  double pregrasp_distance_;
  double contact_height_offset_;
  double lift_height_;
  double cartesian_step_;
  double max_pose_age_;
  double ik_timeout_;
  double grasp_position_tolerance_;
  double grasp_orientation_tolerance_;
  double pregrasp_distance_tolerance_;
  double alternate_face_alignment_tolerance_;
  int maximum_grasp_candidates_;
  double grasp_search_timeout_;
  int ik_attempts_per_candidate_;
  int maximum_planning_candidates_;
  double planning_time_per_candidate_;
  double pregrasp_planning_timeout_;
  int maximum_retry_candidates_;
  double minimum_grasp_joint_margin_;
  double closed_chain_position_tolerance_;
  double closed_chain_orientation_tolerance_;
  double closed_chain_position_step_;
  double closed_chain_orientation_step_;
  int closed_chain_ik_attempts_;
  double closed_chain_ik_timeout_;
  int closed_chain_beam_width_;
  int closed_chain_solutions_per_branch_;
  int closed_chain_projection_limit_;
  double closed_chain_validation_position_step_;
  double closed_chain_validation_orientation_step_;
  double closed_chain_contact_position_error_;
  double closed_chain_contact_orientation_error_;
  double carry_search_timeout_;
  int maximum_carry_candidates_;
  double carry_search_x_range_;
  double carry_search_y_range_;
  double carry_search_z_lower_;
  double carry_search_z_upper_;
  double carry_search_orientation_tolerance_;
  Eigen::Vector3d adaptive_place_position_tolerance_{Eigen::Vector3d::Zero()};
  double adaptive_place_yaw_tolerance_;
  double max_joint_step_;
  bool allow_execution_;
  double velocity_scaling_;
  double acceleration_scaling_;
  Eigen::Isometry3d carry_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d held_box_to_left_contact_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d held_box_to_right_contact_{Eigen::Isometry3d::Identity()};
  bool held_geometry_valid_{false};
  double recovery_position_tolerance_;
  double recovery_angular_tolerance_;
  std::string state_file_;
  std::string initial_state_;
  std::string post_place_named_target_;
  std::string reset_named_target_;
  double reset_preemption_timeout_;
  double reset_state_timeout_;
  double reset_joint_tolerance_;
  double execution_settle_timeout_;
  double execution_joint_tolerance_;
  double execution_velocity_tolerance_;
  int execution_settle_samples_;
  double execution_feedback_max_age_;
  double execution_feedback_future_skew_;
  std::string arm_state_topic_;
  std::map<std::string, double> reset_target_values_;
  bool reset_physical_detach_done_{false};
  bool reset_scene_cleanup_done_{false};
  bool simulate_attachment_;
  std::atomic<bool> physical_attachment_expected_{false};
  Perception3dSource perception_source_{Perception3dSource::NONE};
  std::string depth_filtered_cloud_topic_;
  std::string lidar_filtered_cloud_topic_;
  std::string octomap_clear_service_;
  double octomap_refill_timeout_;
  int octomap_post_clear_samples_;
  double maximum_filtered_cloud_age_;
  double maximum_filtered_cloud_future_skew_;
  double octomap_processing_delay_;
  ResetCoordinator reset_coordinator_;
  std::unique_ptr<ClosedChainPathPlanner> closed_chain_planner_;
  std::mutex execution_transition_mutex_;
  std::mutex execution_error_mutex_;
  std::atomic<bool> execution_cancel_requested_{false};
  ExecuteTrajectoryGoalHandle::SharedPtr active_execution_goal_;
  std::string last_execution_error_;
  std::atomic<uint64_t> hal_arm_feedback_generation_{0};
  std::mutex hal_arm_feedback_mutex_;
  HalArmFeedbackSnapshot latest_hal_arm_feedback_;
  std::atomic<uint8_t> state_{ManipulationState::UNKNOWN};
  std::mutex pose_mutex_;
  std::mutex state_mutex_;
  std::string state_detail_;
  bool have_box_pose_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped latest_box_pose_;
  geometry_msgs::msg::PoseStamped held_pose_;
  std::mutex perception_mutex_;
  std::condition_variable perception_condition_;
  PerceptionSample depth_sample_;
  PerceptionSample lidar_sample_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr box_pose_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_audit_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr
    planning_scene_world_audit_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_filtered_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_filtered_cloud_sub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr hal_arm_state_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr box_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<ManipulationState>::SharedPtr state_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr attach_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detach_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_trajectory_client_;
  rclcpp_action::Server<Pick>::SharedPtr pick_action_server_;
  rclcpp_action::Server<Place>::SharedPtr place_action_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_action_server_;
  rclcpp_action::Server<ResetManipulation>::SharedPtr reset_action_server_;
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
