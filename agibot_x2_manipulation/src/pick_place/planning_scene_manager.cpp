#include "pick_place/planning_scene_manager.hpp"

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation = tf2::toMsg(Eigen::Quaterniond(pose.linear()).normalized());
  return result;
}

}  // namespace

PlanningSceneManager::PlanningSceneManager(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config)
: node_(node), config_(config)
{
  scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description", "x2_pick_place_scene_monitor");
  if (!scene_monitor_->getPlanningScene()) {
    throw std::runtime_error("failed to create MoveIt planning scene monitor");
  }
  scene_monitor_->startStateMonitor();
  scene_monitor_->startSceneMonitor();
  scene_monitor_->startWorldGeometryMonitor();

  scene_audit_sub_ = node_->create_subscription<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", 10,
    [this](const moveit_msgs::msg::PlanningScene::SharedPtr message) {
      for (const auto & object : message->world.collision_objects) {
        auditCollisionObject(object, "/planning_scene");
      }
      for (const auto & attached : message->robot_state.attached_collision_objects) {
        auditCollisionObject(attached.object, "/planning_scene attached object");
      }
    });
  world_audit_sub_ = node_->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
    "/planning_scene_world", 10,
    [this](const moveit_msgs::msg::PlanningSceneWorld::SharedPtr message) {
      for (const auto & object : message->collision_objects) {
        auditCollisionObject(object, "/planning_scene_world");
      }
    });
}

void PlanningSceneManager::auditCollisionObject(
  const moveit_msgs::msg::CollisionObject & object, const char * topic) const
{
  if (object.operation == moveit_msgs::msg::CollisionObject::REMOVE ||
    object.header.frame_id.empty() || object.header.frame_id == config_.planning_frame)
  {
    return;
  }
  if (object.id == config_.box_id) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Planning-scene box '%s' arrived on %s in frame '%s'; expected '%s'. "
      "Stop the conflicting publisher; pick_place_server recreates this object.",
      object.id.c_str(), topic, object.header.frame_id.c_str(), config_.planning_frame.c_str());
    return;
  }
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "External collision object '%s' arrived on %s in frame '%s'. Its publisher must "
    "provide timestamp-compatible TF to planning frame '%s'.",
    object.id.c_str(), topic, object.header.frame_id.c_str(), config_.planning_frame.c_str());
}

bool PlanningSceneManager::synchronize(std::string & error)
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

bool PlanningSceneManager::applyBox(const Eigen::Isometry3d & pose, std::string & error)
{
  try {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = config_.planning_frame;
    object.header.stamp.sec = 0;
    object.header.stamp.nanosec = 0;
    object.id = config_.box_id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {
      config_.dimensions.length, config_.dimensions.width, config_.dimensions.height};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(toPoseMsg(pose));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_interface_.applyCollisionObject(object)) {
      error = "MoveIt rejected the box collision object";
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "failed to apply the box collision object: " + std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::removeBox(std::string & error)
{
  try {
    if (scene_interface_.getObjects({config_.box_id}).empty()) {
      return true;
    }
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = config_.planning_frame;
    object.id = config_.box_id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene_interface_.applyCollisionObject(object)) {
      error = "MoveIt rejected removal of the box collision object";
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "failed to remove the box collision object: " + std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::detachBox(std::string & error)
{
  try {
    if (scene_interface_.getAttachedObjects({config_.box_id}).empty()) {
      return true;
    }
    moveit_msgs::msg::AttachedCollisionObject object;
    object.object.id = config_.box_id;
    object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene_interface_.applyAttachedCollisionObject(object)) {
      error = "MoveIt rejected box detachment from the planning scene";
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "failed to detach the planning-scene box: " + std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::attachBox(std::string & error)
{
  try {
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name = config_.left_tcp;
    object.object.id = config_.box_id;
    object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.touch_links = {
      "left_wrist_roll_link", "left_hand_pad_link", config_.left_tcp,
      "right_wrist_roll_link", "right_hand_pad_link", config_.right_tcp};
    if (!scene_interface_.applyAttachedCollisionObject(object)) {
      error = "MoveIt rejected box attachment to the planning scene";
      return false;
    }
    return verifyBoxState(true, false, error);
  } catch (const std::exception & exception) {
    error = "failed to attach the planning-scene box: " + std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::verifyBoxState(
  bool expect_attached, bool expect_world, std::string & error)
{
  try {
    const auto attached_objects = scene_interface_.getAttachedObjects({config_.box_id});
    const auto world_objects = scene_interface_.getObjects({config_.box_id});
    const bool attached = !attached_objects.empty();
    const bool world = !world_objects.empty();
    if (attached != expect_attached || world != expect_world) {
      error = "planning-scene box state mismatch (attached=" +
        std::string(attached ? "true" : "false") + ", world=" +
        std::string(world ? "true" : "false") + ")";
      return false;
    }
    if (expect_world) {
      const auto object = world_objects.find(config_.box_id);
      if (object == world_objects.end() ||
        object->second.header.frame_id != config_.planning_frame)
      {
        error = "planning-scene box is not expressed in planning frame " + config_.planning_frame;
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

bool PlanningSceneManager::clearBox(std::string & error)
{
  return detachBox(error) && removeBox(error) && verifyBoxState(false, false, error);
}

bool PlanningSceneManager::placeBox(const Eigen::Isometry3d & pose, std::string & error)
{
  return detachBox(error) && applyBox(pose, error) && verifyBoxState(false, true, error);
}

bool PlanningSceneManager::removeWorldBoxTemporarily(
  moveit_msgs::msg::CollisionObject & saved_object, std::string & error)
{
  try {
    const auto objects = scene_interface_.getObjects({config_.box_id});
    const auto found = objects.find(config_.box_id);
    if (found == objects.end()) {
      error = "world box is unavailable for temporary planning-scene removal";
      return false;
    }
    saved_object = found->second;
    return removeBox(error);
  } catch (const std::exception & exception) {
    error = "failed to save the world box before endpoint planning: " +
      std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::restoreWorldBox(
  const moveit_msgs::msg::CollisionObject & saved_object, std::string & error)
{
  try {
    auto object = saved_object;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_interface_.applyCollisionObject(object)) {
      error = "MoveIt rejected restoration of the world box";
      return false;
    }
    return verifyBoxState(false, true, error);
  } catch (const std::exception & exception) {
    error = "failed to restore the world box: " + std::string(exception.what());
    return false;
  }
}

bool PlanningSceneManager::beginVirtualAttachment(
  moveit_msgs::msg::CollisionObject & saved_object, std::string & error)
{
  try {
    const auto objects = scene_interface_.getObjects({config_.box_id});
    const auto found = objects.find(config_.box_id);
    if (found == objects.end()) {
      error = "world box is unavailable for virtual attachment";
      return false;
    }
    saved_object = found->second;
  } catch (const std::exception & exception) {
    error = "failed to save the world box before virtual attachment: " +
      std::string(exception.what());
    return false;
  }
  return attachBox(error);
}

bool PlanningSceneManager::endVirtualAttachment(
  const moveit_msgs::msg::CollisionObject & saved_object, std::string & error)
{
  return detachBox(error) && restoreWorldBox(saved_object, error);
}

bool PlanningSceneManager::collisionFree(
  moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box) const
{
  planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
  collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
  if (ignore_box) {
    acm.setEntry(config_.box_id, true);
  } else if (allow_pad_contact) {
    acm.setEntry(
      config_.box_id, std::vector<std::string>{
        "left_wrist_roll_link", "left_hand_pad_link", config_.left_tcp,
        "right_wrist_roll_link", "right_hand_pad_link", config_.right_tcp}, true);
  }
  collision_detection::CollisionRequest request;
  request.group_name = config_.planning_group;
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state, acm);
  return !result.collision;
}

bool PlanningSceneManager::collisionFreeWithBox(
  moveit::core::RobotState & state, const Eigen::Isometry3d & box_pose,
  bool allow_pad_contact) const
{
  planning_scene_monitor::LockedPlanningSceneRO locked(scene_monitor_);
  auto scene = locked->diff();
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config_.planning_frame;
  object.id = config_.box_id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {
    config_.dimensions.length, config_.dimensions.width, config_.dimensions.height};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(toPoseMsg(box_pose));
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene->processCollisionObjectMsg(object)) {
    return false;
  }
  collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
  if (allow_pad_contact) {
    acm.setEntry(
      config_.box_id, std::vector<std::string>{
        "left_wrist_roll_link", "left_hand_pad_link", config_.left_tcp,
        "right_wrist_roll_link", "right_hand_pad_link", config_.right_tcp}, true);
  }
  collision_detection::CollisionRequest request;
  request.group_name = config_.planning_group;
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state, acm);
  return !result.collision;
}

}  // namespace agibot_x2_manipulation
