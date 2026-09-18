#pragma once

#include <optional>

#include "agibot_x2_manipulation/box_geometry.hpp"
#include "pick_place/pick_place_config.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <set>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{

// Use the existing grasp touch policy only for coordinated disengagement.
planning_scene::PlanningScenePtr retreatContactScene(
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config);

// After operator-confirmed release, clear managed detection geometry.
bool buildResetSceneDiff(
  const planning_scene::PlanningSceneConstPtr & scene, const std::string & box_prefix,
  const std::string & table_id, moveit_msgs::msg::PlanningScene & diff, std::string & error);

struct SceneBox
{
  std::string id;
  BoxDimensions dimensions;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

struct DetectionSceneSnapshot
{
  std::vector<SceneBox> boxes;
  std::optional<SceneBox> table;
};

bool buildDetectionSceneDiff(
  const planning_scene::PlanningSceneConstPtr & scene, const std::string & box_prefix,
  const std::string & table_id, const std::string & frame,
  const DetectionSceneSnapshot & observations, const std::set<std::string> & protected_ids,
  bool confirmed_release, moveit_msgs::msg::PlanningScene & diff, std::string & error);

class PlanningSceneManager
{
public:
  PlanningSceneManager(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config);

  bool synchronize(std::string & error);
  planning_scene::PlanningScenePtr snapshot() const;
  planning_scene::PlanningScenePtr releasedBoxSnapshot(const Eigen::Isometry3d & pose) const;
  bool applyBox(const Eigen::Isometry3d & pose, std::string & error);
  bool applyTable(const Eigen::Isometry3d & pose, std::string & error);
  void publishTableMarker(
    const Eigen::Isometry3d & pose, const builtin_interfaces::msg::Time & stamp);
  bool applyObstacleBoxes(const std::vector<SceneBox> & boxes, std::string & error);
  bool refreshResetBoxes(const std::vector<SceneBox> & boxes, std::string & error);
  bool prepareResetScene(std::string & error);
  bool updateDetectionScene(
    const DetectionSceneSnapshot & observations, const std::set<std::string> & protected_ids,
    bool confirmed_release, std::string & error);
  bool clearOwnedBoxes(std::string & error);
  bool clearManagedBoxes(std::string & error);
  bool removeBox(std::string & error);
  bool detachBox(std::string & error);
  bool attachBox(std::string & error);
  bool verifyBoxState(bool expect_attached, bool expect_world, std::string & error);
  bool clearBox(std::string & error);
  bool placeBox(const Eigen::Isometry3d & pose, std::string & error);
  bool removeWorldBoxTemporarily(
    moveit_msgs::msg::CollisionObject & saved_object, std::string & error);
  bool restoreWorldBox(
    const moveit_msgs::msg::CollisionObject & saved_object, std::string & error);
  bool beginVirtualAttachment(
    moveit_msgs::msg::CollisionObject & saved_object, std::string & error);
  bool endVirtualAttachment(
    const moveit_msgs::msg::CollisionObject & saved_object, std::string & error);
  // When collision_pairs is supplied, collect at most one contact per pair and
  // return their names for diagnostics. Leave it null in high-rate checks.
  bool collisionFree(
    moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box,
    std::string * collision_pairs = nullptr) const;
  bool collisionFreeWithBox(
    moveit::core::RobotState & state, const Eigen::Isometry3d & box_pose,
    bool allow_pad_contact, std::string * collision_pairs = nullptr) const;

private:
  moveit_msgs::msg::CollisionObject makeBoxObject(
    const std::string & id, const BoxDimensions & dimensions,
    const Eigen::Isometry3d & pose) const;
  bool removeOwnedBox(const std::string & id, std::string & error);
  bool isManagedBoxId(const std::string & id) const;
  void auditCollisionObject(
    const moveit_msgs::msg::CollisionObject & object, const char * topic) const;

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  std::string managed_box_id_prefix_;
  moveit::planning_interface::PlanningSceneInterface scene_interface_;
  std::set<std::string> owned_box_ids_;
  std::set<std::string> obstacle_box_ids_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_audit_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr world_audit_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
};

}  // namespace agibot_x2_manipulation
