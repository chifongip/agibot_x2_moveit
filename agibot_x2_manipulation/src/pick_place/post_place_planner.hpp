#pragma once

#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/planning_trace_logger.hpp"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <chrono>
#include <vector>

namespace agibot_x2_manipulation
{

struct HandPosePair
{
  Eigen::Isometry3d left;
  Eigen::Isometry3d right;
};

std::vector<HandPosePair> returnClearanceCandidates(
  const HandPosePair & hands, const Eigen::Vector3d & up, const Eigen::Vector3d & back,
  const PickPlaceConfig & config);

// Checks waypoints and interpolated edges, not just the planner's output samples.
bool validateReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted);
bool validateTimedReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted);

struct PostPlaceSegment
{
  std::string name;
  moveit_msgs::msg::RobotTrajectory trajectory;
  bool retreat{false};
};

struct PostPlacePlan
{
  std::vector<PostPlaceSegment> segments;
};

class PostPlacePlanner
{
public:
  PostPlacePlanner(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    const moveit::core::RobotModelConstPtr & model);

  bool plan(
    const moveit::core::RobotState & start, const HandPosePair & retreat_target,
    const planning_scene::PlanningScenePtr & scene, bool include_retreat,
    PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point outer_deadline =
    std::chrono::steady_clock::time_point::max(), const std::string & named_target = "");
  bool planToNamedTarget(
    const moveit::core::RobotState & start, const planning_scene::PlanningScenePtr & scene,
    const std::string & named_target, PostPlacePlan & output, std::string & error,
    const CancelFunction & canceled);
  bool validateSegment(
    const PostPlaceSegment & segment, const moveit::core::RobotState & current,
    const planning_scene::PlanningScenePtr & scene, std::string & error,
    const CancelFunction & canceled, bool require_empty = false) const;

private:
  using Deadline = std::chrono::steady_clock::time_point;
  planning_scene::PlanningScenePtr contactScene(
    const planning_scene::PlanningScenePtr & scene, bool retreat) const;
  bool endpoint(
    const moveit::core::RobotState & seed, const HandPosePair & poses,
    const planning_scene::PlanningScenePtr & scene, int attempt,
    moveit::core::RobotState & target, const CancelFunction & interrupted) const;
  bool segment(
    const moveit::core::RobotState & start, const moveit::core::RobotState & target,
    const planning_scene::PlanningScenePtr & scene, const std::string & name,
    Deadline deadline, PostPlaceSegment & output, std::string & error,
    const CancelFunction & canceled);
  void trace(const std::string & stage, bool success, const std::string & detail);

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
  PlanningTraceLogger trace_;
};

}  // namespace agibot_x2_manipulation
