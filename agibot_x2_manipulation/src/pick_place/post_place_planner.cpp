#include "pick_place/post_place_planner.hpp"

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
  request.contacts = true;
  request.max_contacts = 16;
  collision_detection::CollisionResult result;
  scene->checkCollision(request, result, state);
  if (!result.collision) {
    return true;
  }
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
  std::string & error, const CancelFunction & interrupted)
{
  if (!validateReturnTrajectory(trajectory, scene, joint_step, error, interrupted)) {
    return false;
  }
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  const auto & joints = message.joint_trajectory;
  joint_trajectory_controller::Trajectory controller;
  robot_trajectory::RobotTrajectory sampled(trajectory.getRobotModel(), trajectory.getGroupName());
  sampled.addSuffixWayPoint(trajectory.getFirstWayPoint(), 0.0);
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
    // identical endpoint positions. The derivative bound also limits joint
    // increments; temporal sampling matches the 100 Hz controller or finer.
    const double step_count = std::max(
      1.0, std::ceil(std::max(duration / 0.01, 10.0 * travel_bound / joint_step)));
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
      moveit::core::RobotState state(trajectory.getFirstWayPoint());
      state.setVariablePositions(joints.joint_names, point.positions);
      sampled.addSuffixWayPoint(state, duration / steps);
    }
  }
  if (!validateReturnTrajectory(sampled, scene, joint_step, error, interrupted)) {
    error = "controller spline invalid: " + error;
    return false;
  }
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
  auto copy = planning_scene::PlanningScene::clone(scene);
  auto & acm = copy->getAllowedCollisionMatrixNonConst();
  acm.setEntry(config_.box_id, false);
  acm.setDefaultEntry(config_.box_id, false);
  if (retreat) {
    acm.setEntry(config_.box_id,
      std::vector<std::string>{"left_hand_pad_link", "right_hand_pad_link", config_.left_tcp,
        config_.right_tcp}, true);
  }
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
  const CancelFunction & canceled)
{
  const auto interrupted = [&]() {return canceled() || std::chrono::steady_clock::now() >= deadline;};
  if (interrupted()) {
    error = "return planning canceled or deadline exhausted";
    return false;
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
    !response.trajectory_ || interrupted())
  {
    error = name + " planning failed, MoveIt code=" + std::to_string(response.error_code_.val);
    trace(name, false, error);
    return false;
  }
  auto & trajectory = *response.trajectory_;
  // Once MoveIt has returned before the search deadline, finish validating that
  // trajectory. Treating a deadline that expires during validation as a
  // collision/spline failure discards a potentially valid safe return route.
  // Cancellation must still interrupt all validation immediately.
  if (!validateReturnTrajectory(trajectory, scene, config_.return_validation_joint_step,
      error, canceled))
  {
    trace(name + "/geometric", false, error);
    return false;
  }
  trace(name + "/geometric", true, "geometric trajectory valid");
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
  std::chrono::steady_clock::time_point outer_deadline, const std::string & named_target)
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
  start.enforceBounds(group);
  start.update();
  moveit::core::RobotState zero(start);
  if (!zero.setToDefaultValues(group, target_name)) {
    error = "return named target unavailable: " + target_name;
    return false;
  }
  moveit::core::RobotState checked_start(start);
  if (!validState(checked_start, release, group, error)) {
    error = "return start invalid: " + error;
    return false;
  }
  if (!zero.satisfiesBounds(group) || !validState(zero, strict, group, error)) {
    error = "return named target invalid: " + error;
    return false;
  }
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d back(-1.0, 0.0, 0.0);
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
  // Retry retreat branches when the complete continuation is infeasible.
  for (int attempt = 0; attempt < config_.return_ik_attempts && !interrupted(); ++attempt) {
    moveit::core::RobotState retreat_end(start);
    PostPlacePlan candidate;
    if (include_retreat) {
      if (!endpoint(start, retreat_target, strict, attempt, retreat_end, interrupted)) {
        continue;
      }
      PostPlaceSegment retreat;
      if (!segment(start, retreat_end, release, "retreat", deadline, retreat, error, canceled)) {
        continue;
      }
      retreat.retreat = true;
      candidate.segments.push_back(std::move(retreat));
    }
    PostPlaceSegment direct;
    if (segment(retreat_end, zero, strict, "zero_direct", deadline, direct, error, canceled)) {
      candidate.segments.push_back(std::move(direct));
      output = std::move(candidate);
      trace("selected", true, "direct return; elapsed=" +
        std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()));
      return true;
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
      PostPlaceSegment last;
      if (!segment(retreat_end, clearance.state, strict, label, deadline, first, error, canceled) ||
        !segment(clearance.state, zero, strict, "zero_from_" + label, deadline, last, error, canceled))
      {
        continue;
      }
      candidate.segments.push_back(std::move(first));
      candidate.segments.push_back(std::move(last));
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

bool PostPlacePlanner::planToNamedTarget(
  const moveit::core::RobotState & start, const planning_scene::PlanningScenePtr & scene,
  const std::string & named_target, PostPlacePlan & output, std::string & error,
  const CancelFunction & canceled)
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
    std::chrono::steady_clock::time_point::max(), named_target);
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
  // Include the measured-to-planned-start edge in validation.
  const auto validation_scene = contactScene(scene, segment.retreat);
  if (!validateTimedReturnTrajectory(trajectory, validation_scene,
      config_.return_validation_joint_step, error, canceled))
  {
    return false;
  }
  trajectory.addPrefixWayPoint(current, 0.0);
  return validateReturnTrajectory(trajectory, validation_scene,
    config_.return_validation_joint_step, error, canceled);
}

}  // namespace agibot_x2_manipulation
