#include "pick_place/post_place_planner.hpp"
#include "pick_place/cartesian_motion.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/endpoint_reached.hpp"
#include <geometric_shapes/shapes.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <joint_trajectory_controller/trajectory.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{

constexpr double kControllerSplineValidationPeriod = 0.02;
constexpr double kControllerSplineSpatialOversampling = 2.0;

double maximumJointDistance(
  const moveit::core::RobotState & a, const moveit::core::RobotState & b,
  const moveit::core::JointModelGroup * group)
{
  double result = 0.0;
  for (const auto * joint : group->getActiveJointModels()) {
    result = std::max(result, joint->distance(a.getJointPositions(joint), b.getJointPositions(joint)));
  }
  return result;
}

bool validState(
  moveit::core::RobotState & state, const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::JointModelGroup * group, std::string & error)
{
  state.update();
  for (const auto & variable : group->getVariableNames()) {
    if (!std::isfinite(state.getVariablePosition(variable))) {
      error = "non-finite return joint position";
      return false;
    }
  }
  collision_detection::CollisionRequest request;
  request.group_name = group->getName();
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state);
  if (!result.collision) {
    return true;
  }
  // Detailed contacts are needed only for rejected states. Keep successful
  // spline samples on the collision checker's cheaper boolean path.
  request.contacts = true;
  request.max_contacts = 16;
  result.clear();
  scene->checkCollision(request, result, state);
  std::ostringstream stream;
  stream << "collision";
  for (const auto & contact : result.contacts) {
    stream << "; " << contact.first.first << " <-> " << contact.first.second;
  }
  error = stream.str();
  return false;
}

std::string poseText(const HandPosePair & poses)
{
  std::ostringstream stream;
  stream << std::setprecision(6);
  for (const auto * pose : {&poses.left, &poses.right}) {
    const Eigen::Quaterniond q(pose->linear());
    stream << "[" << pose->translation().transpose() << "; " <<
      q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "]";
  }
  return stream.str();
}

}  // namespace

std::vector<HandPosePair> returnClearanceCandidates(
  const HandPosePair & hands, const Eigen::Vector3d & up, const Eigen::Vector3d & back,
  const PickPlaceConfig & config)
{
  struct Candidate {double cost; HandPosePair poses;};
  std::vector<Candidate> candidates;
  Eigen::Vector3d outward = hands.left.translation() - hands.right.translation();
  outward -= up * outward.dot(up);
  if (outward.norm() < 1e-6) {
    outward = up.cross(back);
  }
  outward.normalize();
  for (double z : config.return_up_offsets) {
    for (double x : config.return_back_offsets) {
      for (double y : config.return_out_offsets) {
        HandPosePair poses = hands;
        poses.left.translation() += z * up + x * back + y * outward;
        poses.right.translation() += z * up + x * back - y * outward;
        candidates.push_back({z + x + y, poses});
      }
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.cost < b.cost;
  });
  std::vector<HandPosePair> result;
  for (const auto & candidate : candidates) {
    result.push_back(candidate.poses);
  }
  return result;
}

bool validateReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted)
{
  if (!std::isfinite(joint_step) || joint_step <= 0.0 || trajectory.empty() ||
    !trajectory.getGroup())
  {
    error = "empty return trajectory or invalid validation resolution/group";
    return false;
  }
  const auto * group = trajectory.getGroup();
  for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
    const auto & next = trajectory.getWayPoint(index);
    const auto & previous = trajectory.getWayPoint(index == 0 ? 0 : index - 1);
    const double distance = maximumJointDistance(previous, next, group);
    for (const auto & variable : group->getVariableNames()) {
      if (!std::isfinite(next.getVariablePosition(variable))) {
        error = "non-finite return joint position";
        return false;
      }
    }
    const std::size_t steps = std::max<std::size_t>(1, std::ceil(distance / joint_step));
    for (std::size_t sample = 0; sample <= steps; ++sample) {
      if (interrupted()) {
        error = "return trajectory validation interrupted";
        return false;
      }
      moveit::core::RobotState state(previous);
      previous.interpolate(next, static_cast<double>(sample) / steps, state);
      if (!validState(state, scene, group, error)) {
        std::ostringstream detail;
        detail << "edge " << index << " sample " << sample << "/" << steps << ": " << error;
        for (const auto & variable : group->getVariableNames()) {
          detail << " " << variable << "=" << state.getVariablePosition(variable);
        }
        error = detail.str();
        return false;
      }
    }
  }
  return true;
}

bool validateTimedReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted, bool enforce_bounds,
  double minimum_joint_margin,
  const std::function<bool (const moveit::core::RobotState &, std::string &)> & path_valid)
{
  if (!std::isfinite(joint_step) || joint_step <= 0.0 || trajectory.empty() ||
    !trajectory.getGroup())
  {
    error = "empty return trajectory or invalid validation group";
    return false;
  }
  moveit::core::RobotState first(trajectory.getFirstWayPoint());
  if (enforce_bounds && !first.satisfiesBounds(trajectory.getGroup(), 1e-6)) {
    error = "cached trajectory start violates joint position bounds";
    return false;
  }
  if (minimum_joint_margin > 0.0 &&
    first.getMinDistanceToPositionBounds(trajectory.getGroup()).first + 1e-12 < minimum_joint_margin)
  {
    error = "cached trajectory start violates minimum joint margin";
    return false;
  }
  if (!validState(first, scene, trajectory.getGroup(), error)) {
    error = "return trajectory start invalid: " + error;
    return false;
  }
  if (path_valid && !path_valid(first, error)) {return false;}
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  const auto & joints = message.joint_trajectory;
  if (joints.joint_names.empty() || joints.points.empty()) {
    error = "return trajectory has no joint samples";
    return false;
  }
  joint_trajectory_controller::Trajectory controller;
  const auto * group = trajectory.getGroup();
  moveit::core::RobotState state(trajectory.getFirstWayPoint());
  for (std::size_t index = 1; index < joints.points.size(); ++index) {
    const auto & a = joints.points[index - 1];
    const auto & b = joints.points[index];
    const rclcpp::Time time_a(rclcpp::Duration(a.time_from_start).nanoseconds(), RCL_ROS_TIME);
    const rclcpp::Time time_b(rclcpp::Duration(b.time_from_start).nanoseconds(), RCL_ROS_TIME);
    const double duration = (time_b - time_a).seconds();
    if (!std::isfinite(duration) || duration <= 0.0) {
      error = "return trajectory has non-increasing timestamps";
      return false;
    }
    double travel_bound = 0.0;
    for (std::size_t joint = 0; joint < joints.joint_names.size(); ++joint) {
      double bound = std::abs(b.positions[joint] - a.positions[joint]);
      for (const auto * point : {&a, &b}) {
        if (!point->velocities.empty()) {
          if (point->velocities.size() != joints.joint_names.size() ||
            !std::isfinite(point->velocities[joint]))
          {
            error = "invalid return trajectory velocity";
            return false;
          }
          bound += std::abs(point->velocities[joint]) * duration;
        }
        if (!point->accelerations.empty()) {
          if (point->accelerations.size() != joints.joint_names.size() ||
            !std::isfinite(point->accelerations[joint]))
          {
            error = "invalid return trajectory acceleration";
            return false;
          }
          bound += std::abs(point->accelerations[joint]) * duration * duration;
        }
      }
      travel_bound = std::max(travel_bound, bound);
    }
    // Sample the controller's cubic/quintic spline, including overshoot with
    // identical endpoint positions. The derivative bound limits the joint
    // increment to at most half the configured geometric validation step.
    // This is deliberately less dense than the controller update rate: MoveIt
    // has already validated the OMPL path, and this pass covers interpolation
    // that the controller adds between those waypoints.
    const double step_count = std::max(
      1.0, std::ceil(std::max(
        duration / kControllerSplineValidationPeriod,
        kControllerSplineSpatialOversampling * travel_bound / joint_step)));
    if (!std::isfinite(step_count) || step_count > 1000000.0) {
      error = "return controller spline exceeds validation sample budget";
      return false;
    }
    const auto steps = static_cast<std::size_t>(step_count);
    for (std::size_t sample = 1; sample <= steps; ++sample) {
      if (interrupted()) {
        error = "controller spline validation interrupted";
        return false;
      }
      trajectory_msgs::msg::JointTrajectoryPoint point;
      const auto sample_time = time_a + rclcpp::Duration::from_seconds(
        duration * static_cast<double>(sample) / steps);
      controller.interpolate_between_points(time_a, a, time_b, b, sample_time, point);
      if (point.positions.size() != joints.joint_names.size() ||
        !std::all_of(point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}))
      {
        error = "controller spline invalid: non-finite or incomplete joint position";
        return false;
      }
      state.setVariablePositions(joints.joint_names, point.positions);
      if (enforce_bounds && !state.satisfiesBounds(group, 1e-6)) {
        error = "cached controller spline violates joint position bounds";
        return false;
      }
      if (minimum_joint_margin > 0.0 &&
        state.getMinDistanceToPositionBounds(group).first + 1e-12 < minimum_joint_margin)
      {
        error = "cached controller spline violates minimum joint margin";
        return false;
      }
      if (!validState(state, scene, group, error)) {
        error = "controller spline invalid: segment " + std::to_string(index) +
          " sample " + std::to_string(sample) + "/" + std::to_string(steps) + ": " + error;
        return false;
      }
      if (path_valid && !path_valid(state, error)) {return false;}
    }
  }
  return true;
}

static bool validateReusableTrajectory(
  moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotState & planned_start, const moveit::core::RobotState & current,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  std::string & error, const CancelFunction & interrupted,
  const Eigen::Isometry3d * box_to_left = nullptr,
  const Eigen::Isometry3d * box_to_right = nullptr, const Eigen::Isometry3d * target_pose = nullptr)
{
  if (interrupted()) {error = "cached trajectory validation canceled"; return false;}
  if (!scene || planned_start.getRobotModel() != current.getRobotModel()) {
    error = "cached trajectory scene or robot model unavailable";
    return false;
  }
  const auto * group = current.getJointModelGroup(config.planning_group);
  if (!group || message.joint_trajectory.points.empty() ||
    !message.multi_dof_joint_trajectory.joint_names.empty())
  {
    error = "cached trajectory is empty or unsupported";
    return false;
  }
  const auto & names = message.joint_trajectory.joint_names;
  const std::set<std::string> commanded(names.begin(), names.end());
  const auto & variables = group->getVariableNames();
  if (commanded.size() != names.size() ||
    commanded != std::set<std::string>(variables.begin(), variables.end()))
  {
    error = "cached trajectory does not command the complete planning group";
    return false;
  }
  const bool carry = box_to_left != nullptr;
  std::vector<const moveit::core::AttachedBody *> attached;
  for (const auto * state : {&current, &scene->getCurrentState()}) {
    state->getAttachedBodies(attached);
    if (!carry && !attached.empty()) {error = "cached Pick trajectory requires empty arms"; return false;}
    if (carry) {
      if (attached.size() != 1U || attached.front()->getName() != config.box_id ||
        attached.front()->getAttachedLinkName() != config.left_tcp ||
        attached.front()->getShapes().size() != 1U ||
        attached.front()->getShapePosesInLinkFrame().size() != 1U)
      {
        error = "cached carry attachment identity or geometry mismatch";
        return false;
      }
      const auto * shape = dynamic_cast<const shapes::Box *>(attached.front()->getShapes().front().get());
      const auto expected = box_to_left->inverse();
      const auto & actual = attached.front()->getShapePosesInLinkFrame().front();
      if (!shape || std::abs(shape->size[0] - config.dimensions.length) > 1e-9 ||
        std::abs(shape->size[1] - config.dimensions.width) > 1e-9 ||
        std::abs(shape->size[2] - config.dimensions.height) > 1e-9 ||
        !endpointReached(actual, actual, expected, expected))
      {
        error = "cached carry attachment dimensions or grasp transform mismatch";
        return false;
      }
    }
    attached.clear();
  }
  if (!carry) {
    planned_start.getAttachedBodies(attached);
    if (!attached.empty()) {error = "cached Pick trajectory requires empty arms"; return false;}
  }
  const double tolerance = config.execution_joint_tolerance;
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    error = "invalid cached trajectory start tolerance";
    return false;
  }
  for (const auto & name : current.getRobotModel()->getVariableNames()) {
    const double measured = current.getVariablePosition(name);
    const double planned = planned_start.getVariablePosition(name);
    if (!std::isfinite(measured) || !std::isfinite(planned) ||
      std::abs(measured - planned) > (commanded.count(name) ? tolerance : 0.001))
    {
      error = "measured state differs from cached Pick start: " + name;
      return false;
    }
  }
  const auto & first = message.joint_trajectory.points.front().positions;
  if (first.size() != names.size()) {error = "cached trajectory start is incomplete"; return false;}
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (!std::isfinite(first[i]) || std::abs(first[i] - current.getVariablePosition(names[i])) > tolerance) {
      error = "measured joints differ from cached trajectory start";
      return false;
    }
  }
  for (const auto & point : message.joint_trajectory.points) {
    if (point.positions.size() != names.size() ||
      !std::all_of(point.positions.begin(), point.positions.end(), [](double value) {return std::isfinite(value);}) ||
      (!point.velocities.empty() && point.velocities.size() != names.size()) ||
      (!point.accelerations.empty() && point.accelerations.size() != names.size()))
    {
      error = "cached trajectory contains incomplete or non-finite joint data";
      return false;
    }
  }
  robot_trajectory::RobotTrajectory trajectory(current.getRobotModel(), config.planning_group);
  trajectory.setRobotTrajectoryMsg(current, message);
  const bool rebase = std::any_of(names.begin(), names.end(), [&](const std::string & name) {
      return std::abs(current.getVariablePosition(name) -
             trajectory.getFirstWayPoint().getVariablePosition(name)) > 1e-6;
    });
  if (rebase) {
    // Do not execute a spline starting at the old planned positions. Replace
    // its start with measured feedback and regenerate timing before validating
    // the full controller interpolation, bounds, and collisions.
    *trajectory.getFirstWayPointPtr() = current;
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(config.return_path_tolerance);
    if (!timing.computeTimeStamps(trajectory, config.velocity_scaling, config.acceleration_scaling)) {
      error = "cached Pick trajectory retiming failed";
      return false;
    }
  }
  const auto contact = carry ? graspContactScene(scene, config) : retreatContactScene(scene, config);
  if (!validateTimedReturnTrajectory(trajectory, contact, config.return_validation_joint_step,
      error, interrupted, true, carry ? config.minimum_carry_joint_margin : 0.0)) {return false;}
  robot_trajectory::RobotTrajectory edge(current.getRobotModel(), config.planning_group);
  edge.addSuffixWayPoint(current, 0.0);
  edge.addSuffixWayPoint(trajectory.getFirstWayPoint(), 0.0);
  if (!validateReturnTrajectory(edge, contact, config.return_validation_joint_step, error, interrupted)) {
    return false;
  }
  if (carry) {
    const auto & end = trajectory.getLastWayPoint();
    const auto accuracy = [&](const Eigen::Isometry3d & actual, const Eigen::Isometry3d & target) {
        return actual.matrix().allFinite() && target.matrix().allFinite() &&
          (actual.translation() - target.translation()).norm() <= config.closed_chain_contact_position_error &&
          std::abs(Eigen::AngleAxisd(actual.linear().transpose() * target.linear()).angle()) <=
          config.closed_chain_contact_orientation_error;
      };
    if (!accuracy(end.getGlobalLinkTransform(config.left_tcp), *target_pose * *box_to_left) ||
      !accuracy(end.getGlobalLinkTransform(config.right_tcp), *target_pose * *box_to_right))
    {
      error = "cached carry endpoint violates TCP accuracy";
      return false;
    }
  }
  if (rebase) {trajectory.getRobotTrajectoryMsg(message);}
  error.clear();
  return true;
}

bool validateReusablePickTrajectory(
  moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotState & planned_start, const moveit::core::RobotState & current,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  std::string & error, const CancelFunction & interrupted)
{
  return validateReusableTrajectory(message, planned_start, current, scene, config, error, interrupted);
}

bool copySceneAttachments(
  moveit::core::RobotState & target, const moveit::core::RobotState & scene_state)
{
  if (&target == &scene_state) {return true;}
  std::vector<const moveit::core::AttachedBody *> bodies;
  scene_state.getAttachedBodies(bodies);
  for (const auto * body : bodies) {
    if (!target.getRobotModel()->hasLinkModel(body->getAttachedLinkName())) {return false;}
  }
  target.clearAttachedBodies();
  for (const auto * body : bodies) {
    target.attachBody(body->getName(), body->getPose(), body->getShapes(), body->getShapePoses(),
      body->getTouchLinks(), body->getAttachedLinkName(), body->getDetachPosture(), body->getSubframes());
  }
  target.update();
  return true;
}

bool validateReusableCarryTrajectory(
  moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotState & planned_start, const moveit::core::RobotState & current,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  const Eigen::Isometry3d & box_to_left, const Eigen::Isometry3d & box_to_right,
  const Eigen::Isometry3d & target_pose, std::string & error, const CancelFunction & interrupted)
{
  // MoveGroup feedback commonly contains joint positions without attached bodies.
  // The synchronized scene owns attachment geometry; never validate carry without it.
  std::vector<const moveit::core::AttachedBody *> measured_bodies;
  current.getAttachedBodies(measured_bodies);
  if (scene && measured_bodies.empty()) {
    moveit::core::RobotState measured(current);
    if (!copySceneAttachments(measured, scene->getCurrentState())) {
      error = "cached carry attachment link is absent from the feedback robot model";
      return false;
    }
    return validateReusableTrajectory(message, planned_start, measured, scene, config, error,
      interrupted, &box_to_left, &box_to_right, &target_pose);
  }
  return validateReusableTrajectory(message, planned_start, current, scene, config, error,
    interrupted, &box_to_left, &box_to_right, &target_pose);
}

bool tryDirectJointTrajectory(
  const moveit::core::RobotState & start, const moveit::core::RobotState & target,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  double minimum_joint_margin, moveit_msgs::msg::RobotTrajectory & output,
  std::string & error, const CancelFunction & interrupted)
{
  if (!scene || interrupted()) {error = "direct joint route unavailable or interrupted"; return false;}
  const auto * group = start.getJointModelGroup(config.planning_group);
  moveit::core::RobotState reference(start), endpoint(target);
  if (!group || target.getRobotModel() != start.getRobotModel() ||
    !normalizePlanningStart(reference, group, config.place_start_state_bounds_tolerance) ||
    !target.satisfiesBounds(group))
  {
    error = "direct joint route model or bounds mismatch";
    return false;
  }
  for (const auto & name : group->getVariableNames()) {
    if (!std::isfinite(start.getVariablePosition(name)) ||
      !std::isfinite(target.getVariablePosition(name)))
    {
      error = "direct joint route contains non-finite positions";
      return false;
    }
  }
  if (!copySceneAttachments(reference, scene->getCurrentState()) ||
    !copySceneAttachments(endpoint, scene->getCurrentState()))
  {
    error = "direct joint route attachment mismatch";
    return false;
  }
  robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), config.planning_group);
  trajectory.addSuffixWayPoint(reference, 0.0);
  trajectory.addSuffixWayPoint(endpoint, 0.0);
  trajectory_processing::TimeOptimalTrajectoryGeneration timing(config.return_path_tolerance);
  if (!timing.computeTimeStamps(trajectory, config.velocity_scaling, config.acceleration_scaling) ||
    interrupted())
  {
    error = "direct joint route timing failed or interrupted";
    return false;
  }
  // TOTG can sample its endpoint twice when duration is a multiple of its
  // resampling period. Keep the final derivatives and validate the resulting
  // controller spline; do not merge distinct states at the same timestamp.
  moveit_msgs::msg::RobotTrajectory timed;
  trajectory.getRobotTrajectoryMsg(timed);
  auto & points = timed.joint_trajectory.points;
  if (points.size() >= 3U) {
    const auto & previous = points[points.size() - 2U];
    const auto & last = points.back();
    bool duplicate = previous.time_from_start == last.time_from_start &&
      previous.positions.size() == last.positions.size();
    for (std::size_t index = 0; duplicate && index < last.positions.size(); ++index) {
      duplicate = std::abs(previous.positions[index] - last.positions[index]) <= 1e-9;
    }
    if (duplicate) {
      points.erase(points.end() - 2);
      trajectory.setRobotTrajectoryMsg(reference, timed);
    }
  }
  if (!validateTimedReturnTrajectory(trajectory, scene, config.return_validation_joint_step,
      error, interrupted, true, minimum_joint_margin)) {return false;}
  if (interrupted() || !jointEndpointReached(trajectory.getFirstWayPoint(), reference, group, 1e-6) ||
    !jointEndpointReached(trajectory.getLastWayPoint(), target, group, 1e-6))
  {
    error = "direct joint route endpoint changed or interrupted";
    return false;
  }
  trajectory.getRobotTrajectoryMsg(output);
  error.clear();
  return true;
}

PostPlacePlanner::PostPlacePlanner(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
  const moveit::core::RobotModelConstPtr & model)
: node_(node), config_(config),
  trace_(node->get_logger(), config.planning_log_file, config.planning_log_directory)
{
  // No start-state repair or time adapter: validate the geometric path first,
  // then explicitly run TOTG and check its final output on the same scene.
  pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
    model, node, "post_place_ompl", "ompl_interface/OMPLPlanner", std::vector<std::string>{});
  const auto manager = pipeline_->getPlannerManager();
  if (!manager) {
    throw std::runtime_error("post-place OMPL planner could not be loaded");
  }
  auto configurations = manager->getPlannerConfigurations();
  planning_interface::PlannerConfigurationSettings settings;
  settings.name = config.planning_group;
  settings.group = config.planning_group;
  settings.config["type"] = "geometric::RRTConnect";
  settings.config["longest_valid_segment_fraction"] =
    std::to_string(config.return_longest_valid_segment_fraction);
  configurations[settings.name] = settings;
  settings.name = config.planning_group + "[ReturnRRTConnect]";
  configurations[settings.name] = settings;
  manager->setPlannerConfigurations(configurations);
  pipeline_->displayComputedMotionPlans(false);
}

void PostPlacePlanner::trace(const std::string & stage, bool success, const std::string & detail)
{
  if (trace_.enabled()) {
    trace_.write(node_->now().nanoseconds(), "post_place_return", success, detail, {{"stage", stage}});
  }
}

planning_scene::PlanningScenePtr PostPlacePlanner::contactScene(
  const planning_scene::PlanningScenePtr & scene, bool retreat) const
{
  if (retreat) {
    return retreatContactScene(scene, config_);
  }
  auto copy = planning_scene::PlanningScene::clone(scene);
  auto & acm = copy->getAllowedCollisionMatrixNonConst();
  acm.setEntry(config_.box_id, false);
  acm.setDefaultEntry(config_.box_id, false);
  return copy;
}

bool PostPlacePlanner::endpoint(
  const moveit::core::RobotState & seed, const HandPosePair & poses,
  const planning_scene::PlanningScenePtr & scene, int attempt,
  moveit::core::RobotState & target, const CancelFunction & interrupted) const
{
  const auto * group = seed.getJointModelGroup(config_.planning_group);
  target = seed;
  if (attempt > 0) {
    target.setToRandomPositions(group);
  }
  for (const auto & arm : {
      std::make_pair(config_.left_group_name, std::make_pair(config_.left_tcp, poses.left)),
      std::make_pair(config_.right_group_name, std::make_pair(config_.right_tcp, poses.right))})
  {
    if (interrupted()) {
      return false;
    }
    target.update();
    const auto * arm_group = target.getJointModelGroup(arm.first);
    if (!arm_group || !arm_group->getSolverInstance() ||
      !target.setFromIK(arm_group, arm.second.second,
        arm.second.first, std::min(0.05, config_.closed_chain_ik_timeout)))
    {
      return false;
    }
  }
  target.update();
  for (const auto & hand : {std::make_pair(config_.left_tcp, poses.left),
      std::make_pair(config_.right_tcp, poses.right)})
  {
    const auto & actual = target.getGlobalLinkTransform(hand.first);
    if ((actual.translation() - hand.second.translation()).norm() > 0.005 ||
      Eigen::Quaterniond(actual.linear()).angularDistance(
        Eigen::Quaterniond(hand.second.linear())) > 0.02)
    {
      return false;
    }
  }
  std::string error;
  if (!target.satisfiesBounds(group)) {
    return false;
  }
  return validState(target, scene, group, error);
}

bool PostPlacePlanner::segment(
  const moveit::core::RobotState & start, const moveit::core::RobotState & target,
  const planning_scene::PlanningScenePtr & scene, const std::string & name,
  Deadline deadline, PostPlaceSegment & output, std::string & error,
  const CancelFunction & canceled, bool allow_no_motion)
{
  error.clear();
  for (int attempt = 0; attempt < config_.return_planning_attempts; ++attempt) {
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      if (error.empty()) {
        error = "return planning canceled or deadline exhausted";
      }
      return false;
    }
    const auto stage = name + "/attempt_" + std::to_string(attempt + 1);
    trace(stage, true, "starting trajectory planning attempt");
    PostPlaceSegment candidate;
    std::string attempt_error;
    // Retry the entire pipeline, including timing and controller spline checks.
    // Every attempt shares the original deadline and collision scene.
    if (segmentOnce(start, target, scene, name, deadline, candidate, attempt_error, canceled, allow_no_motion)) {
      output = std::move(candidate);
      error.clear();
      return true;
    }
    error = std::move(attempt_error);
    trace(stage, false, error);
    if (attempt + 1 < config_.return_planning_attempts && !canceled() &&
      std::chrono::steady_clock::now() < deadline)
    {
      RCLCPP_WARN(node_->get_logger(),
        "Return segment %s attempt %d/%d rejected: %s; replanning",
        name.c_str(), attempt + 1, config_.return_planning_attempts, error.c_str());
    }
  }
  return false;
}

bool PostPlacePlanner::segmentOnce(
  const moveit::core::RobotState & start, const moveit::core::RobotState & target,
  const planning_scene::PlanningScenePtr & scene, const std::string & name,
  Deadline deadline, PostPlaceSegment & output, std::string & error,
  const CancelFunction & canceled, bool allow_no_motion)
{
  const auto interrupted = [&]() {return canceled() || std::chrono::steady_clock::now() >= deadline;};
  if (interrupted()) {
    error = "return planning canceled or deadline exhausted";
    return false;
  }
  const auto * group = start.getJointModelGroup(config_.planning_group);
  if (allow_no_motion && config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
    jointEndpointReached(start, target, group, 1e-6))
  {
    moveit::core::RobotState checked(start);
    if (!checked.satisfiesBounds(group) || !validState(checked, scene, group, error)) {return false;}
    robot_trajectory::RobotTrajectory stationary(start.getRobotModel(), config_.planning_group);
    stationary.addSuffixWayPoint(start, 0.0);
    stationary.addSuffixWayPoint(start, 0.02);
    output.name = name;
    output.no_motion = true;
    stationary.getRobotTrajectoryMsg(output.trajectory);
    trace(name, true, "validated no-motion segment; OMPL skipped");
    return true;
  }
  if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
    moveit_msgs::msg::RobotTrajectory direct;
    std::string direct_error;
    if (tryDirectJointTrajectory(start, target, scene, config_, 0.0, direct, direct_error, interrupted)) {
      output.name = name;
      output.trajectory = std::move(direct);
      trace(name + "/direct", true, "validated joint route; OMPL skipped");
      return true;
    }
    trace(name + "/direct", false, direct_error);
  }
  planning_interface::MotionPlanRequest request;
  request.group_name = config_.planning_group;
  request.planner_id = "ReturnRRTConnect";
  request.num_planning_attempts = 1;
  request.allowed_planning_time = std::min(config_.return_planning_time_per_attempt,
    std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count());
  request.max_velocity_scaling_factor = config_.velocity_scaling;
  request.max_acceleration_scaling_factor = config_.acceleration_scaling;
  moveit::core::robotStateToRobotStateMsg(start, request.start_state);
  request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      target, target.getJointModelGroup(config_.planning_group), 1e-4));
  planning_interface::MotionPlanResponse response;
  // Terminate the active context on action cancellation, not just after plan().
  std::atomic<bool> finished{false};
  std::thread watcher([&]() {
    while (!finished.load()) {
      if (interrupted()) {
        pipeline_->terminate();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  bool planned = false;
  try {
    planned = pipeline_->generatePlan(scene, request, response);
  } catch (...) {
    finished.store(true);
    watcher.join();
    throw;
  }
  finished.store(true);
  watcher.join();
  if (!planned || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
    !response.trajectory_ || canceled())
  {
    error = name + " planning failed, MoveIt code=" + std::to_string(response.error_code_.val);
    trace(name, false, error);
    return false;
  }
  auto & trajectory = *response.trajectory_;
  // A successful MoveIt response is a complete route candidate. Treating a
  // deadline that expires during subsequent processing as a collision/spline
  // failure discards a potentially valid safe return route. Cancellation must
  // still interrupt all validation immediately.
  trajectory_processing::TimeOptimalTrajectoryGeneration timing(config_.return_path_tolerance);
  if (!timing.computeTimeStamps(trajectory, config_.velocity_scaling, config_.acceleration_scaling)) {
    error = name + " time parameterization failed";
    return false;
  }
  if (!validateTimedReturnTrajectory(trajectory, scene, config_.return_validation_joint_step,
      error, canceled) ||
    maximumJointDistance(trajectory.getFirstWayPoint(), start, trajectory.getGroup()) > 1e-3 ||
    maximumJointDistance(trajectory.getLastWayPoint(), target, trajectory.getGroup()) >
    std::min(1e-3, config_.reset_joint_tolerance))
  {
    if (error.empty()) {
      error = name + " processed trajectory endpoint mismatch";
    }
    trace(name + "/processed", false, error);
    return false;
  }
  trace(name + "/processed", true, "processed trajectory valid");
  output.name = name;
  trajectory.getRobotTrajectoryMsg(output.trajectory);
  return true;
}

bool PostPlacePlanner::plan(
  const moveit::core::RobotState & supplied_start, const HandPosePair & retreat_target,
  const planning_scene::PlanningScenePtr & scene, bool include_retreat,
  PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
  std::chrono::steady_clock::time_point outer_deadline, const std::string & named_target,
  const std::string & intermediate_target)
{
  output.segments.clear();
  error.clear();
  if (canceled()) {
    error = "return search canceled";
    return false;
  }
  // A hypothetical release may receive the endpoint of an attached-box plan.
  // The released box is now a fixed world obstacle, never an attached body.
  moveit::core::RobotState start(supplied_start);
  start.clearAttachedBody(config_.box_id);
  const auto began = std::chrono::steady_clock::now();
  const auto deadline = std::min(outer_deadline,
    began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.return_planning_timeout)));
  const auto interrupted = [&]() {return canceled() || std::chrono::steady_clock::now() >= deadline;};
  const auto * group = start.getJointModelGroup(config_.planning_group);
  auto strict = contactScene(scene, false);
  auto release = contactScene(scene, include_retreat);
  const auto & target_name = named_target.empty() ? config_.post_place_named_target : named_target;
  if (!group) {
    error = "return planning group unavailable: " + config_.planning_group;
    return false;
  }
  // Device feedback can lie outside the model limits.  Keep the reported
  // state for execution-start matching, but plan from its bounded model
  // representation so MoveIt does not reject the return request.
  const bool allow_no_motion = start.satisfiesBounds(group);
  start.enforceBounds(group);
  start.update();
  moveit::core::RobotState target_state(start);
  if (!target_state.setToDefaultValues(group, target_name)) {
    error = "return named target unavailable: " + target_name;
    return false;
  }
  moveit::core::RobotState checked_start(start);
  if (!validState(checked_start, release, group, error)) {
    error = "return start invalid: " + error;
    return false;
  }
  if (!target_state.satisfiesBounds(group) || !validState(target_state, strict, group, error)) {
    error = "return named target invalid: " + error;
    return false;
  }
  moveit::core::RobotState intermediate(target_state);
  if (!intermediate_target.empty()) {
    if (!intermediate.setToDefaultValues(group, intermediate_target)) {
      error = "return intermediate target unavailable: " + intermediate_target;
      return false;
    }
    intermediate.update();
    if (!intermediate.satisfiesBounds(group) || !validState(intermediate, strict, group, error)) {
      error = "return intermediate target invalid: " + intermediate_target + ": " + error;
      return false;
    }
  }
  const auto append_named = [&](const moveit::core::RobotState & from,
      const std::string & label, PostPlacePlan & candidate) {
      PostPlaceSegment first;
      if (!segment(from, intermediate, strict,
          intermediate_target.empty() ? label : "to_" + intermediate_target,
          deadline, first, error, canceled, allow_no_motion))
      {
        return false;
      }
      if (!intermediate_target.empty()) {
        PostPlaceSegment last;
        if (!segment(intermediate, target_state, strict, "from_" + intermediate_target + "_to_" +
            target_name, deadline, last, error, canceled, allow_no_motion))
        {
          return false;
        }
        candidate.segments.push_back(std::move(first));
        candidate.segments.push_back(std::move(last));
      } else {
        candidate.segments.push_back(std::move(first));
      }
      return true;
    };
  const bool direct_pose_to_pose =
    config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE;
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d back(-1.0, 0.0, 0.0);
  if (!direct_pose_to_pose) {
    const auto table = strict->getWorld()->getObject(config_.table_collision_id);
    if (table && !table->global_shape_poses_.empty()) {
      const auto & pose = table->global_shape_poses_.front();
      up = pose.linear().col(2);
      back = -pose.translation();
      back -= up * back.dot(up);
      if (back.norm() < 1e-6) {
        back = -pose.linear().col(0);
      }
      back.normalize();
    }
  }
  // Pose-to-pose mode has a single requested retreat endpoint. Closed-chain
  // mode retains its alternate IK and clearance search because its object
  // constraint can make the direct return infeasible.
  const int branch_attempts = direct_pose_to_pose ? 1 : config_.return_ik_attempts;
  for (int attempt = 0; attempt < branch_attempts && !interrupted(); ++attempt) {
    moveit::core::RobotState retreat_end(start);
    PostPlacePlan candidate;
    if (include_retreat) {
      PostPlaceSegment retreat;
      if (direct_pose_to_pose) {
        retreat.name = "retreat";
        const bool planned = planCartesianMotion(start, retreat_target, release, config_,
          retreat.trajectory, retreat_end, error, canceled, deadline);
        if (trace_.enabled()) {
          trace_.write(node_->now().nanoseconds(), "cartesian_segment", planned, error,
            {{"segment", "retreat"}});
        }
        if (!planned) {continue;}
        if (!validState(retreat_end, strict, retreat_end.getJointModelGroup(config_.planning_group), error)) {
          continue;
        }
      } else {
        if (!endpoint(start, retreat_target, strict, attempt, retreat_end, interrupted) ||
          !segment(start, retreat_end, release, "retreat", deadline, retreat, error, canceled, allow_no_motion))
        {continue;}
      }
      retreat.retreat = true;
      candidate.segments.push_back(std::move(retreat));
    }
    if (append_named(retreat_end, target_name + "_direct", candidate)) {
      output = std::move(candidate);
      trace("selected", true, "direct return; elapsed=" +
        std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()));
      return true;
    }
    if (direct_pose_to_pose) {
      break;
    }
    HandPosePair hands{retreat_end.getGlobalLinkTransform(config_.left_tcp),
      retreat_end.getGlobalLinkTransform(config_.right_tcp)};
    const auto poses = returnClearanceCandidates(hands, up, back, config_);
    struct ClearanceEndpoint
    {
      HandPosePair poses;
      moveit::core::RobotState state;
      std::size_t index;
      int seed;
      double score;
    };
    std::vector<ClearanceEndpoint> endpoints;
    const auto ik_deadline = std::min(deadline, std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.return_planning_timeout * 0.15)));
    const auto ik_interrupted = [&]() {
        return interrupted() || std::chrono::steady_clock::now() >= ik_deadline;
      };
    // Preserve distinct IK branches before planning. Round-robin candidate
    // poses during collection so failed IK does not starve larger clearances.
    for (int seed = 0; seed < config_.return_ik_attempts && !ik_interrupted(); ++seed) {
      for (std::size_t index = 0; index < poses.size() && !ik_interrupted(); ++index) {
        moveit::core::RobotState target(retreat_end);
        if (!endpoint(retreat_end, poses[index], strict, seed, target, ik_interrupted)) {
          trace("clearance_" + std::to_string(index) + "_seed_" + std::to_string(seed), false,
            "IK/bounds/collision rejection; poses=" + poseText(poses[index]));
          continue;
        }
        const bool duplicate = std::any_of(endpoints.begin(), endpoints.end(),
          [&](const ClearanceEndpoint & stored) {
            return stored.index == index && maximumJointDistance(stored.state, target, group) < 0.01;
          });
        if (!duplicate) {
          // Interleave branches and pose offsets; prefer small joint travel.
          endpoints.push_back({poses[index], target, index, seed,
            static_cast<double>(index) + 3.0 * seed + 0.1 * target.distance(retreat_end, group)});
        }
      }
    }
    std::stable_sort(endpoints.begin(), endpoints.end(),
      [](const ClearanceEndpoint & a, const ClearanceEndpoint & b) {return a.score < b.score;});
    for (const auto & clearance : endpoints) {
      if (interrupted()) {
        break;
      }
      const auto label = "clearance_" + std::to_string(clearance.index) + "_seed_" +
        std::to_string(clearance.seed) + "_retreat_" + std::to_string(attempt);
      PostPlaceSegment first;
      PostPlacePlan continuation;
      if (!segment(retreat_end, clearance.state, strict, label, deadline, first, error, canceled) ||
        !append_named(clearance.state, target_name + "_from_" + label, continuation))
      {
        continue;
      }
      candidate.segments.push_back(std::move(first));
      candidate.segments.insert(candidate.segments.end(),
        continuation.segments.begin(), continuation.segments.end());
      output = std::move(candidate);
      trace("selected", true, label + "; poses=" + poseText(clearance.poses) + "; elapsed=" +
        std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()));
      return true;
    }
  }
  error = (canceled() ? "return search canceled" : interrupted() ? "return search deadline exhausted" :
    "no valid return route found") + std::string("; last failure: ") + error;
  trace("exhausted", false, error);
  return false;
}

bool PostPlacePlanner::planRetreat(
  const moveit::core::RobotState & start, const HandPosePair & target,
  const planning_scene::PlanningScenePtr & scene, PostPlacePlan & output,
  std::string & error, const CancelFunction & canceled, Deadline deadline)
{
  output.segments.clear();
  error.clear();
  const auto release = contactScene(scene, true);
  const auto strict = contactScene(scene, false);
  const auto * group = start.getJointModelGroup(config_.planning_group);
  if (!group) {error = "retreat planning group unavailable"; return false;}
  auto checked_start = start;
  if (!validState(checked_start, release, group, error)) {
    error = "retreat start invalid: " + error;
    return false;
  }
  auto end = start;
  PostPlaceSegment retreat;
  retreat.name = "retreat";
  retreat.retreat = true;
  const bool planned = planCartesianMotion(start, target, release, config_, retreat.trajectory,
    end, error, canceled, deadline);
  if (trace_.enabled()) {
    trace_.write(node_->now().nanoseconds(), "cartesian_segment", planned, error,
      {{"segment", "retreat"}});
  }
  if (!planned) {return false;}
  if (!validState(end, strict, group, error)) {
    error = "retreat endpoint has not cleared the placed box: " + error;
    return false;
  }
  output.segments.push_back(std::move(retreat));
  return true;
}

bool PostPlacePlanner::planToNamedTarget(
  const moveit::core::RobotState & start, const planning_scene::PlanningScenePtr & scene,
  const std::string & named_target, PostPlacePlan & output, std::string & error,
  const CancelFunction & canceled, std::chrono::steady_clock::time_point deadline)
{
  output.segments.clear();
  if (named_target.empty()) {
    error = "empty named target";
    return false;
  }
  std::vector<const moveit::core::AttachedBody *> attached;
  start.getAttachedBodies(attached);
  if (attached.empty()) {
    scene->getCurrentState().getAttachedBodies(attached);
  }
  if (!attached.empty()) {
    error = "empty-arm named-target planning requires confirmed release; attached object: " +
      attached.front()->getName();
    return false;
  }
  return plan(start, {}, scene, false, output, error, canceled,
    deadline, named_target);
}

bool PostPlacePlanner::validateSegment(
  const PostPlaceSegment & segment, const moveit::core::RobotState & supplied_current,
  const planning_scene::PlanningScenePtr & scene, std::string & error,
  const CancelFunction & canceled, bool require_empty) const
{
  if (require_empty) {
    std::vector<const moveit::core::AttachedBody *> attached;
    supplied_current.getAttachedBodies(attached);
    if (attached.empty()) {
      scene->getCurrentState().getAttachedBodies(attached);
    }
    if (!attached.empty()) {
      error = "reset execution blocked by attached object: " + attached.front()->getName();
      return false;
    }
  }
  moveit::core::RobotState current(supplied_current);
  current.clearAttachedBody(config_.box_id);
  robot_trajectory::RobotTrajectory trajectory(current.getRobotModel(), config_.planning_group);
  trajectory.setRobotTrajectoryMsg(current, segment.trajectory);
  if (trajectory.empty() || maximumJointDistance(current, trajectory.getFirstWayPoint(),
      trajectory.getGroup()) > config_.execution_joint_tolerance)
  {
    error = "measured return start differs from planned start";
    return false;
  }
  if (segment.no_motion &&
    (!current.satisfiesBounds(trajectory.getGroup(), config_.place_start_state_bounds_tolerance) ||
    !jointEndpointReached(current, trajectory.getLastWayPoint(), trajectory.getGroup(),
      config_.execution_joint_tolerance)))
  {
    error = "measured state moved since no-motion return planning";
    return false;
  }
  // Include the measured-to-planned-start edge in validation.
  const auto validation_scene = contactScene(scene, segment.retreat);
  if (!validateTimedReturnTrajectory(trajectory, validation_scene,
      config_.return_validation_joint_step, error, canceled))
  {
    return false;
  }
  if (segment.retreat) {
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      const auto & first = trajectory.getFirstWayPoint();
      const auto & last = trajectory.getLastWayPoint();
      const HandPosePair from{first.getGlobalLinkTransform(config_.left_tcp),
        first.getGlobalLinkTransform(config_.right_tcp)};
      const HandPosePair to{last.getGlobalLinkTransform(config_.left_tcp),
        last.getGlobalLinkTransform(config_.right_tcp)};
      const HandPosePair measured{current.getGlobalLinkTransform(config_.left_tcp),
        current.getGlobalLinkTransform(config_.right_tcp)};
      if (!validateCartesianState(measured, from, to, config_.cartesian_path_position_tolerance,
          config_.cartesian_path_orientation_tolerance, error) ||
        !validateCartesianTrajectory(trajectory, validation_scene, config_, from, to,
          error, canceled)) {return false;}
    }
    moveit::core::RobotState retreat_end(trajectory.getLastWayPoint());
    if (!validState(retreat_end, contactScene(scene, false), trajectory.getGroup(), error)) {
      error = "retreat endpoint has not cleared the placed box: " + error;
      return false;
    }
  }
  robot_trajectory::RobotTrajectory start_edge(current.getRobotModel(), config_.planning_group);
  start_edge.addSuffixWayPoint(current, 0.0);
  start_edge.addSuffixWayPoint(trajectory.getFirstWayPoint(), 0.0);
  return validateReturnTrajectory(start_edge, validation_scene,
    config_.return_validation_joint_step, error, canceled);
}

}  // namespace agibot_x2_manipulation
