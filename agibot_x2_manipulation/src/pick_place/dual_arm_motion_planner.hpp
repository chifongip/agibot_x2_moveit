#pragma once

#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/closed_chain_path_planner.hpp"
#include "pick_place/pick_place_config.hpp"
#include "pick_place/planning_scene_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace agibot_x2_manipulation
{

struct PlannedGrasp
{
  GraspCandidate candidate;
  double joint_limit_margin{0.0};
  double joint_distance{0.0};
};

using CarryRoute = ClosedChainRoute;

struct AdaptiveCarryPlan
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  CarryRoute route{CarryRoute::DIRECT};
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::shared_ptr<moveit::core::RobotState> end_state;
};

using CancelFunction = std::function<bool ()>;
using ContinuationFunction = std::function<bool (
      const moveit::core::RobotState &, const PlannedGrasp &, std::string &)>;

class DualArmMotionPlanner
{
public:
  DualArmMotionPlanner(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    moveit::planning_interface::MoveGroupInterface & move_group,
    PlanningSceneManager & planning_scene,
    Eigen::Isometry3d & held_box_to_left_contact,
    Eigen::Isometry3d & held_box_to_right_contact,
    bool & held_geometry_valid, geometry_msgs::msg::PoseStamped & held_pose);
  ~DualArmMotionPlanner();

  DualArmMotionPlanner(const DualArmMotionPlanner &) = delete;
  DualArmMotionPlanner & operator=(const DualArmMotionPlanner &) = delete;

  bool buildApproach(
    const moveit::core::RobotState & start, const GraspGeometry & target,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    const CancelFunction & canceled);
  GraspGeometry graspFromBoxToTcp(
    const Eigen::Isometry3d & box_pose,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    double pregrasp_distance) const;
  bool buildCarryRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & target_pose, CarryRoute route, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled);
  bool planAdaptiveCarry(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    bool plan_only, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
    std::string & error, const CancelFunction & canceled);
  bool planAdaptivePlace(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & requested_pose, bool from_pick, bool ignore_box,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    Eigen::Isometry3d & selected_pose, std::string & error,
    const CancelFunction & canceled);
  bool planPickPath(
    const geometry_msgs::msg::PoseStamped & box_message, const Eigen::Isometry3d & pick_pose,
    moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
    moveit_msgs::msg::RobotTrajectory & approach, moveit::core::RobotState & contact_end,
    PlannedGrasp & selected, const ContinuationFunction & continuation,
    std::string & error, const CancelFunction & canceled);
  void updateHeldPoseFromRobot();
  bool validateHeldClosure(std::string & error);
  void clearGraspMarkers();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agibot_x2_manipulation
