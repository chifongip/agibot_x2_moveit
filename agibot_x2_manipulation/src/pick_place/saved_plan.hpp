#pragma once

#include "pick_place/cartesian_motion.hpp"
#include "pick_place/box_pose_tracker.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{

enum class SavedStepKind {MOTION, ATTACH, RELEASE};

struct SavedStep
{
  std::string name;
  SavedStepKind kind{SavedStepKind::MOTION};
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::shared_ptr<moveit::core::RobotState> start;
  std::vector<CartesianSegment> cartesian;
  bool held{false};
  bool contact{false};
  bool retreat{false};
};

struct SavedPlan
{
  std::string id;
  std::string action;
  PickPlaceConfig config;
  uint64_t profile_version{0};
  std::string instance_id;
  std::string profile_id;
  std::vector<TrackedBoxPose> boxes;
  std::optional<Eigen::Isometry3d> table_tag;
  std::string table_profile_id;
  uint64_t table_profile_version{0};
  std::map<std::string, Eigen::Isometry3d> table_tags;
  Eigen::Isometry3d box_to_left{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d box_to_right{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d pick_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d carry_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d place_pose{Eigen::Isometry3d::Identity()};
  // Empty frame denotes a table-derived request; explicit poses are resolved
  // once into the planning frame, before adaptive placement corrections.
  std::optional<geometry_msgs::msg::PoseStamped> placement_request;
  uint8_t carry_target{0};
  moveit_msgs::msg::PlanningSceneWorld world;
  std::vector<SavedStep> steps;
};

bool validate_table_detection(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & reference,
  const PickPlaceConfig & config, std::string & error);

bool validateSavedCheckpointState(
  const moveit::core::RobotState & measured, const planning_scene::PlanningScenePtr & scene,
  const PickPlaceConfig & config, std::string & error);

// Saved execution accepts configured start error without constructing motion.
// CONTINUOUS retains legacy connectors; VERIFY_START never retimes the path.
enum class SavedMotionPreparation {CONTINUOUS, SEPARATE, VERIFY_START};

struct SavedAlignmentInfo
{
  double start_difference{0.0};
  double timing_scale{1.0};
  std::optional<moveit_msgs::msg::RobotTrajectory> alignment;
  std::string strategy{"none"};
  std::string fallback_reason;
};

bool prepareSavedMotion(
  const SavedStep & step, const SavedPlan & plan, const moveit::core::RobotState & measured,
  const planning_scene::PlanningScenePtr & scene, moveit_msgs::msg::RobotTrajectory & output,
  double & alignment_seconds, std::string & error, const CancelFunction & canceled,
  SavedAlignmentInfo * alignment_info = nullptr,
  SavedMotionPreparation preparation = SavedMotionPreparation::CONTINUOUS);

class SavedPlanStore
{
public:
  void put(std::shared_ptr<SavedPlan> plan) {plans_[plan->action] = std::move(plan);}
  std::shared_ptr<SavedPlan> claim(const std::string & id, const std::string & action)
  {
    const auto found = plans_.find(action);
    if (found == plans_.end() || found->second->id != id) {return nullptr;}
    auto result = found->second;
    plans_.clear();
    return result;
  }
  void clear() {plans_.clear();}

private:
  // The server's operation reservation serializes all access.
  std::map<std::string, std::shared_ptr<SavedPlan>> plans_;
};

}  // namespace agibot_x2_manipulation
