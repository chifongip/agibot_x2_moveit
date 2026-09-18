#include "pick_place/planning_scene_manager.hpp"

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <exception>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

void formatCollisionPairs(
  const collision_detection::CollisionResult & result, std::string * collision_pairs)
{
  if (!collision_pairs) {
    return;
  }
  collision_pairs->clear();
  if (!result.collision) {
    return;
  }

  std::ostringstream stream;
  bool first_pair = true;
  for (const auto & [pair, contacts] : result.contacts) {
    if (contacts.empty()) {
      continue;
    }
    if (!first_pair) {
      stream << "; ";
    }
    stream << pair.first << " <-> " << pair.second;
    first_pair = false;
  }
  if (first_pair) {
    stream << "contact pair unavailable";
  }
  *collision_pairs = stream.str();
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

// Allow only the held/grasped box to touch the wrist and hand geometry.  All
// wrist collisions with the rest of the robot and the environment remain
// enabled in the planning scene.
std::vector<std::string> boxTouchLinks(const PickPlaceConfig & config)
{
  return {
    "left_wrist_yaw_link", "left_wrist_pitch_link", "left_wrist_roll_link",
    "left_hand_pad_link", config.left_tcp,
    "right_wrist_yaw_link", "right_wrist_pitch_link", "right_wrist_roll_link",
    "right_hand_pad_link", config.right_tcp};
}

}  // namespace

planning_scene::PlanningScenePtr retreatContactScene(
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config)
{
  auto copy = planning_scene::PlanningScene::clone(scene);
  copy->getCurrentStateNonConst().clearAttachedBody(config.box_id);
  auto & acm = copy->getAllowedCollisionMatrixNonConst();
  acm.setEntry(config.box_id, false);
  acm.setDefaultEntry(config.box_id, false);
  acm.setEntry(config.box_id, boxTouchLinks(config), true);
  return copy;
}

bool buildResetSceneDiff(
  const planning_scene::PlanningSceneConstPtr & scene, const std::string & box_prefix,
  const std::string & table_id, moveit_msgs::msg::PlanningScene & diff, std::string & error)
{
  diff = moveit_msgs::msg::PlanningScene();
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  if (!scene) {
    error = "reset scene unavailable";
    return false;
  }
  auto released = planning_scene::PlanningScene::clone(scene);
  std::set<std::string> removed;
  const auto managed = [&box_prefix](const std::string & id) {
      return id == box_prefix || id.rfind(box_prefix + "_", 0) == 0;
    };
  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached;
  scene->getAttachedCollisionObjectMsgs(attached);
  for (const auto & object : attached) {
    if (!managed(object.object.id)) {
      continue;
    }
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.link_name = object.link_name;
    detach.object.id = object.object.id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!released->processAttachedCollisionObjectMsg(detach)) {
      error = "cannot preserve released reset box: " + object.object.id;
      return false;
    }
    diff.robot_state.attached_collision_objects.push_back(detach);
    removed.insert(object.object.id);
  }
  auto & acm = released->getAllowedCollisionMatrixNonConst();
  for (const auto & id : released->getWorld()->getObjectIds()) {
    if (managed(id) || id == table_id) {
      removed.insert(id);
    }
  }
  for (const auto & id : removed) {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    diff.world.collision_objects.push_back(object);
    acm.removeEntry(id);
    acm.setDefaultEntry(id, false);
  }
  acm.getMessage(diff.allowed_collision_matrix);
  return true;
}

bool buildDetectionSceneDiff(
  const planning_scene::PlanningSceneConstPtr & scene, const std::string & box_prefix,
  const std::string & table_id, const std::string & frame,
  const DetectionSceneSnapshot & observations, const std::set<std::string> & protected_ids,
  bool confirmed_release, moveit_msgs::msg::PlanningScene & diff, std::string & error)
{
  diff = moveit_msgs::msg::PlanningScene();
  const auto managed = [&box_prefix](const std::string & id) {
      return id == box_prefix || id.rfind(box_prefix + "_", 0) == 0;
    };
  std::vector<SceneBox> objects = observations.boxes;
  if (observations.table) {
    if (observations.table->id != table_id) {
      error = "unexpected detection table ID";
      return false;
    }
    objects.push_back(*observations.table);
  }
  std::set<std::string> ids;
  for (const auto & box : objects) {
    const auto & rotation = box.pose.linear();
    if ((!managed(box.id) && box.id != table_id) || box.id.empty() ||
      !ids.insert(box.id).second || protected_ids.count(box.id) != 0U ||
      !box.pose.matrix().allFinite() ||
      !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(rotation.determinant() - 1.0) > 1e-6 ||
      !std::isfinite(box.dimensions.length) || box.dimensions.length <= 0.0 ||
      !std::isfinite(box.dimensions.width) || box.dimensions.width <= 0.0 ||
      !std::isfinite(box.dimensions.height) || box.dimensions.height <= 0.0)
    {
      error = "invalid detection snapshot object: " + box.id;
      return false;
    }
  }
  if (!scene) {
    error = "detection scene unavailable";
    return false;
  }
  auto protected_objects = protected_ids;
  if (!confirmed_release) {
    std::vector<const moveit::core::AttachedBody *> attached;
    scene->getCurrentState().getAttachedBodies(attached);
    for (const auto * body : attached) {
      protected_objects.insert(body->getName());
      if (ids.count(body->getName()) != 0U) {
        error = "detection snapshot would overwrite attached object: " + body->getName();
        return false;
      }
    }
  }
  if (confirmed_release) {
    if (!protected_objects.empty()) {
      error = "confirmed release cannot protect task objects";
      return false;
    }
    if (!buildResetSceneDiff(scene, box_prefix, table_id, diff, error)) {
      return false;
    }
    collision_detection::AllowedCollisionMatrix acm(diff.allowed_collision_matrix);
    for (const auto & id : ids) {
      acm.setEntry(id, false);
      acm.setDefaultEntry(id, false);
    }
    acm.getMessage(diff.allowed_collision_matrix);
  } else {
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    auto acm = scene->getAllowedCollisionMatrix();
    for (const auto & id : scene->getWorld()->getObjectIds()) {
      if ((managed(id) || id == table_id) && protected_objects.count(id) == 0U) {
        moveit_msgs::msg::CollisionObject removal;
        removal.id = id;
        removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        diff.world.collision_objects.push_back(removal);
        acm.removeEntry(id);
        acm.setDefaultEntry(id, false);
      }
    }
    // Ordinary detections never inherit grasp contact allowances.
    for (const auto & id : ids) {
      acm.setEntry(id, false);
      acm.setDefaultEntry(id, false);
    }
    acm.getMessage(diff.allowed_collision_matrix);
  }
  for (const auto & box : objects) {
    moveit_msgs::msg::CollisionObject object;
    object.id = box.id;
    object.header.frame_id = frame;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box.dimensions.length, box.dimensions.width, box.dimensions.height};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(toPoseMsg(box.pose));
    diff.world.collision_objects.push_back(object);
  }
  return true;
}

bool PlanningSceneManager::updateDetectionScene(
  const DetectionSceneSnapshot & observations, const std::set<std::string> & protected_ids,
  bool confirmed_release, std::string & error)
{
  if (!synchronize(error)) {
    return false;
  }
  moveit_msgs::msg::PlanningScene diff;
  if (!buildDetectionSceneDiff(snapshot(), managed_box_id_prefix_, config_.table_collision_id,
      config_.planning_frame, observations, protected_ids, confirmed_release, diff, error))
  {
    return false;
  }
  if (!scene_interface_.applyPlanningScene(diff)) {
    error = "MoveIt rejected detection scene update";
    return false;
  }
  if (!synchronize(error)) {
    return false;
  }
  owned_box_ids_ = protected_ids;
  obstacle_box_ids_.clear();
  for (const auto & box : observations.boxes) {
    owned_box_ids_.insert(box.id);
    obstacle_box_ids_.insert(box.id);
  }
  for (const auto & object : diff.world.collision_objects) {
    const bool observed = std::any_of(observations.boxes.begin(), observations.boxes.end(),
      [&object](const SceneBox & box) {return box.id == object.id;}) ||
      (observations.table && observations.table->id == object.id);
    RCLCPP_DEBUG(node_->get_logger(), "Detection scene %s: %s (%s)",
      object.operation == moveit_msgs::msg::CollisionObject::REMOVE ? "remove" : "add/update",
      object.id.c_str(), observed ? "fresh replacement" : "absent/stale detection");
  }
  return true;
}

moveit_msgs::msg::CollisionObject PlanningSceneManager::makeBoxObject(
  const std::string & id, const BoxDimensions & dimensions,
  const Eigen::Isometry3d & pose) const
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config_.planning_frame;
  object.header.stamp.sec = 0;
  object.header.stamp.nanosec = 0;
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {dimensions.length, dimensions.width, dimensions.height};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(toPoseMsg(pose));
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

PlanningSceneManager::PlanningSceneManager(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config)
: node_(node), config_(config), managed_box_id_prefix_(config.box_id)
{
  scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description", "x2_pick_place_scene_monitor");
  if (!scene_monitor_->getPlanningScene()) {
    throw std::runtime_error("failed to create MoveIt planning scene monitor");
  }
  scene_monitor_->startStateMonitor();
  scene_monitor_->startSceneMonitor();
  const bool load_octomap_monitor =
    config_.perception_source != Perception3dSource::NONE;
  scene_monitor_->startWorldGeometryMonitor(
    planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
    planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
    load_octomap_monitor);

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
  table_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/table_markers", rclcpp::QoS(1).transient_local());
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
    const auto object = makeBoxObject(config_.box_id, config_.dimensions, pose);
    if (!scene_interface_.applyCollisionObject(object)) {
      error = "MoveIt rejected the box collision object";
      return false;
    }
    owned_box_ids_.insert(config_.box_id);
    return true;
  } catch (const std::exception & exception) {
    error = "failed to apply the box collision object: " + std::string(exception.what());
    return false;
  }
}

planning_scene::PlanningScenePtr PlanningSceneManager::snapshot() const
{
  planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
  return planning_scene::PlanningScene::clone(scene);
}

planning_scene::PlanningScenePtr PlanningSceneManager::releasedBoxSnapshot(
  const Eigen::Isometry3d & pose) const
{
  auto scene = snapshot();
  scene->getCurrentStateNonConst().clearAttachedBody(config_.box_id);
  if (!scene->processCollisionObjectMsg(makeBoxObject(config_.box_id, config_.dimensions, pose))) {
    throw std::runtime_error("cannot place released box in return-planning snapshot");
  }
  return scene;
}

bool PlanningSceneManager::applyTable(const Eigen::Isometry3d & pose, std::string & error)
{
  if (!config_.table_collision_enabled) {
    return true;
  }
  try {
    const auto object = makeBoxObject(
      config_.table_collision_id, config_.table_dimensions, pose);
    if (!scene_interface_.applyCollisionObject(object)) {
      error = "MoveIt rejected the table collision object";
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "failed to apply the table collision object: " + std::string(exception.what());
    return false;
  }
}

void PlanningSceneManager::publishTableMarker(
  const Eigen::Isometry3d & pose, const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.planning_frame;
  marker.header.stamp = stamp;
  marker.ns = "collision_table";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = toPoseMsg(pose);
  marker.scale.x = config_.table_dimensions.length;
  marker.scale.y = config_.table_dimensions.width;
  marker.scale.z = config_.table_dimensions.height;
  marker.color.r = 0.85F;
  marker.color.g = 0.55F;
  marker.color.b = 0.15F;
  marker.color.a = 0.35F;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(marker);
  table_marker_pub_->publish(markers);
}

bool PlanningSceneManager::removeOwnedBox(const std::string & id, std::string & error)
{
  try {
    if (!scene_interface_.getAttachedObjects({id}).empty()) {
      moveit_msgs::msg::AttachedCollisionObject attached;
      attached.object.id = id;
      attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!scene_interface_.applyAttachedCollisionObject(attached)) {
        error = "MoveIt rejected removal of owned attached box '" + id + "'";
        return false;
      }
    }
    if (!scene_interface_.getObjects({id}).empty()) {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = config_.planning_frame;
      object.id = id;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!scene_interface_.applyCollisionObject(object)) {
        error = "MoveIt rejected removal of owned box '" + id + "'";
        return false;
      }
    }
    return true;
  } catch (const std::exception & exception) {
    error = "failed to remove owned box '" + id + "': " + exception.what();
    return false;
  }
}

bool PlanningSceneManager::applyObstacleBoxes(
  const std::vector<SceneBox> & boxes, std::string & error)
{
  std::set<std::string> requested_ids;
  for (const auto & box : boxes) {
    if (box.id.empty() || box.id == config_.box_id ||
      box.dimensions.length <= 0.0 || box.dimensions.width <= 0.0 ||
      box.dimensions.height <= 0.0 || !box.pose.matrix().allFinite())
    {
      error = "invalid visible-box obstacle";
      return false;
    }
    if (!requested_ids.insert(box.id).second) {
      error = "duplicate visible-box obstacle ID: " + box.id;
      return false;
    }
  }

  for (auto it = obstacle_box_ids_.begin(); it != obstacle_box_ids_.end();) {
    if (requested_ids.count(*it) != 0U) {
      ++it;
      continue;
    }
    if (!removeOwnedBox(*it, error)) {
      return false;
    }
    owned_box_ids_.erase(*it);
    it = obstacle_box_ids_.erase(it);
  }

  for (const auto & box : boxes) {
    try {
      if (!scene_interface_.applyCollisionObject(
          makeBoxObject(box.id, box.dimensions, box.pose)))
      {
        error = "MoveIt rejected visible-box obstacle '" + box.id + "'";
        return false;
      }
    } catch (const std::exception & exception) {
      error = "failed to apply visible-box obstacle '" + box.id + "': " + exception.what();
      return false;
    }
    owned_box_ids_.insert(box.id);
    obstacle_box_ids_.insert(box.id);
  }
  return true;
}

bool PlanningSceneManager::clearOwnedBoxes(std::string & error)
{
  for (const auto & id : owned_box_ids_) {
    if (!removeOwnedBox(id, error)) {
      return false;
    }
  }
  owned_box_ids_.clear();
  obstacle_box_ids_.clear();
  return true;
}

bool PlanningSceneManager::prepareResetScene(std::string & error)
{
  if (!synchronize(error)) {
    return false;
  }
  moveit_msgs::msg::PlanningScene diff;
  if (!buildResetSceneDiff(snapshot(), managed_box_id_prefix_, config_.table_collision_id,
      diff, error) ||
    !scene_interface_.applyPlanningScene(diff))
  {
    if (error.empty()) {
      error = "MoveIt rejected reset scene preparation";
    }
    return false;
  }
  owned_box_ids_.clear();
  obstacle_box_ids_.clear();
  return synchronize(error);
}

bool PlanningSceneManager::refreshResetBoxes(
  const std::vector<SceneBox> & boxes, std::string & error)
{
  std::set<std::string> ids;
  // Validate the entire batch before updating any obstacle. Reset preparation
  // clears the previous detection snapshot before these fresh observations.
  for (const auto & box : boxes) {
    if (!isManagedBoxId(box.id) || !ids.insert(box.id).second ||
      !box.pose.matrix().allFinite() ||
      !std::isfinite(box.dimensions.length) || box.dimensions.length <= 0.0 ||
      !std::isfinite(box.dimensions.width) || box.dimensions.width <= 0.0 ||
      !std::isfinite(box.dimensions.height) || box.dimensions.height <= 0.0)
    {
      error = "invalid reset box observation";
      return false;
    }
  }
  for (const auto & box : boxes) {
    if (!scene_interface_.applyCollisionObject(makeBoxObject(box.id, box.dimensions, box.pose))) {
      error = "MoveIt rejected reset box observation: " + box.id;
      return false;
    }
    owned_box_ids_.insert(box.id);
    obstacle_box_ids_.insert(box.id);
  }
  return true;
}

bool PlanningSceneManager::isManagedBoxId(const std::string & id) const
{
  return id == managed_box_id_prefix_ ||
         id.rfind(managed_box_id_prefix_ + "_", 0) == 0;
}

bool PlanningSceneManager::clearManagedBoxes(std::string & error)
{
  std::set<std::string> managed_ids = owned_box_ids_;
  managed_ids.insert(config_.box_id);
  try {
    for (const auto & entry : scene_interface_.getObjects()) {
      if (isManagedBoxId(entry.first)) {
        managed_ids.insert(entry.first);
      }
    }
    for (const auto & entry : scene_interface_.getAttachedObjects()) {
      if (isManagedBoxId(entry.first)) {
        managed_ids.insert(entry.first);
      }
    }
  } catch (const std::exception & exception) {
    error = "failed to discover managed collision objects: " +
      std::string(exception.what());
    return false;
  }

  for (const auto & id : managed_ids) {
    if (!removeOwnedBox(id, error)) {
      return false;
    }
  }
  owned_box_ids_.clear();
  obstacle_box_ids_.clear();
  return true;
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
    object.touch_links = boxTouchLinks(config_);
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
  return clearManagedBoxes(error) && verifyBoxState(false, false, error);
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
  moveit::core::RobotState & state, bool allow_pad_contact, bool ignore_box,
  std::string * collision_pairs) const
{
  planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
  collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
  if (ignore_box) {
    acm.setEntry(config_.box_id, true);
  } else if (allow_pad_contact) {
    acm.setEntry(
      config_.box_id, boxTouchLinks(config_), true);
  }
  collision_detection::CollisionRequest request;
  request.group_name = config_.planning_group;
  if (collision_pairs) {
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
  }
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state, acm);
  formatCollisionPairs(result, collision_pairs);
  return !result.collision;
}

bool PlanningSceneManager::collisionFreeWithBox(
  moveit::core::RobotState & state, const Eigen::Isometry3d & box_pose,
  bool allow_pad_contact, std::string * collision_pairs) const
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
      config_.box_id, boxTouchLinks(config_), true);
  }
  collision_detection::CollisionRequest request;
  request.group_name = config_.planning_group;
  if (collision_pairs) {
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
  }
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state, acm);
  formatCollisionPairs(result, collision_pairs);
  return !result.collision;
}

}  // namespace agibot_x2_manipulation
