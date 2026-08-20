#pragma once

#include "pick_place/pick_place_config.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <string>

namespace agibot_x2_manipulation
{

class PlanningSceneManager
{
public:
  PlanningSceneManager(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config);

  bool synchronize(std::string & error);
  bool applyBox(const Eigen::Isometry3d & pose, std::string & error);
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
  bool collisionFree(
    moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box) const;
  bool collisionFreeWithBox(
    moveit::core::RobotState & state, const Eigen::Isometry3d & box_pose,
    bool allow_pad_contact) const;

private:
  void auditCollisionObject(
    const moveit_msgs::msg::CollisionObject & object, const char * topic) const;

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  moveit::planning_interface::PlanningSceneInterface scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_audit_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr world_audit_sub_;
};

}  // namespace agibot_x2_manipulation
