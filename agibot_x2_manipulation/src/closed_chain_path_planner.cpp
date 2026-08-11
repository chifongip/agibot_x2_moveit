#include "agibot_x2_manipulation/closed_chain_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

double rotationDistance(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  const Eigen::Quaterniond qa(a.linear());
  const Eigen::Quaterniond qb(b.linear());
  return 2.0 * std::acos(std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
}

Eigen::Isometry3d interpolate(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double t)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = from.translation() + t * (to.translation() - from.translation());
  result.linear() = Eigen::Quaterniond(from.linear()).slerp(
    t, Eigen::Quaterniond(to.linear())).normalized().toRotationMatrix();
  return result;
}

double jointDistance(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double squared = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double delta = a[i] - b[i];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

struct Branch
{
  ClosedChainPath path;
  std::vector<double> last;
  std::vector<double> previous_delta;
  Eigen::Isometry3d last_pose{Eigen::Isometry3d::Identity()};
};

}  // namespace

const char * closedChainFailureName(ClosedChainFailure failure)
{
  switch (failure) {
    case ClosedChainFailure::NONE: return "none";
    case ClosedChainFailure::CANCELED: return "canceled";
    case ClosedChainFailure::DEADLINE: return "deadline";
    case ClosedChainFailure::IK: return "ik";
    case ClosedChainFailure::BOUNDS: return "bounds";
    case ClosedChainFailure::COLLISION: return "collision";
    case ClosedChainFailure::CONTINUITY: return "continuity";
    case ClosedChainFailure::CONTACT: return "contact";
  }
  return "unknown";
}

const char * closedChainRouteName(ClosedChainRoute route)
{
  switch (route) {
    case ClosedChainRoute::DIRECT: return "direct";
    case ClosedChainRoute::LOW_XY_THEN_LIFT: return "low_xy_then_lift";
    case ClosedChainRoute::LIFT_THEN_XY: return "lift_then_xy";
    case ClosedChainRoute::ROTATE_BEFORE_TRANSLATION: return "rotate_before_translation";
    case ClosedChainRoute::ROTATE_AFTER_TRANSLATION: return "rotate_after_translation";
    case ClosedChainRoute::DOGLEG_NEGATIVE_Y: return "dogleg_negative_y";
    case ClosedChainRoute::DOGLEG_POSITIVE_Y: return "dogleg_positive_y";
  }
  return "unknown";
}

std::vector<ClosedChainWaypoint> makePlaceRouteWaypoints(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & place,
  double lift_height, double dogleg_y, bool from_pick, ClosedChainRoute route)
{
  if (lift_height < 0.0 || dogleg_y < 0.0) {
    throw std::invalid_argument("place route distances must be nonnegative");
  }
  Eigen::Isometry3d start_lift = from;
  if (from_pick) {
    start_lift.translation().z() += lift_height;
  }
  Eigen::Isometry3d place_lift = place;
  place_lift.translation().z() += lift_height;
  std::vector<ClosedChainWaypoint> controls{{from, false, "place_start"}};
  const auto add = [&controls](const Eigen::Isometry3d & pose, const char * segment) {
      controls.push_back({pose, true, segment});
    };
  if (route == ClosedChainRoute::DIRECT) {
    if (from_pick) {
      add(start_lift, "pick_lift");
    }
    add(place_lift, "move_to_place_lift");
  } else if (route == ClosedChainRoute::LOW_XY_THEN_LIFT) {
    Eigen::Isometry3d low = place;
    low.translation().z() = from.translation().z();
    low.linear() = from.linear();
    add(low, "low_xy_translation");
    add(place_lift, "lift_at_place");
  } else if (route == ClosedChainRoute::LIFT_THEN_XY) {
    Eigen::Isometry3d high = start_lift;
    high.translation().z() = std::max(high.translation().z(), place_lift.translation().z());
    Eigen::Isometry3d high_target = place_lift;
    high_target.translation().z() = high.translation().z();
    high_target.linear() = high.linear();
    add(high, "lift_before_translation");
    add(high_target, "high_xy_translation");
    add(place_lift, "place_lift_rotation");
  } else if (route == ClosedChainRoute::ROTATE_BEFORE_TRANSLATION) {
    Eigen::Isometry3d rotated = start_lift;
    rotated.linear() = place.linear();
    if (from_pick) {
      add(start_lift, "pick_lift");
    }
    add(rotated, "rotate_before_translation");
    add(place_lift, "place_translation");
  } else if (route == ClosedChainRoute::ROTATE_AFTER_TRANSLATION) {
    Eigen::Isometry3d translated = place_lift;
    translated.linear() = start_lift.linear();
    if (from_pick) {
      add(start_lift, "pick_lift");
    }
    add(translated, "place_translation");
    add(place_lift, "rotate_after_translation");
  } else {
    const Eigen::Isometry3d route_start = from_pick ? start_lift : from;
    Eigen::Isometry3d dogleg = interpolate(route_start, place_lift, 0.5);
    dogleg.translation().y() +=
      route == ClosedChainRoute::DOGLEG_NEGATIVE_Y ? -dogleg_y : dogleg_y;
    if (from_pick) {
      add(start_lift, "pick_lift");
    }
    add(dogleg, "bounded_place_dogleg");
    add(place_lift, "dogleg_to_place");
  }
  add(place, "place_descent");
  return controls;
}

ClosedChainPathPlanner::ClosedChainPathPlanner(ClosedChainPlannerConfig config)
: config_(std::move(config))
{
  if (config_.beam_width == 0 || config_.solutions_per_branch == 0 ||
    config_.projection_limit == 0 || config_.position_tolerance < 0.0 ||
    config_.orientation_tolerance < 0.0 || config_.position_step <= 0.0 ||
    config_.orientation_step <= 0.0 || config_.validation_position_step <= 0.0 ||
    config_.validation_orientation_step <= 0.0 || config_.maximum_joint_step <= 0.0)
  {
    throw std::invalid_argument("invalid closed-chain planner configuration");
  }
}

std::vector<Eigen::Isometry3d> ClosedChainPathPlanner::projections(
  const Eigen::Isometry3d & nominal) const
{
  std::vector<Eigen::Vector3d> translations{Eigen::Vector3d::Zero()};
  for (double value = config_.position_step;
    value <= config_.position_tolerance + 1e-12; value += config_.position_step)
  {
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      offset[axis] = value;
      translations.push_back(offset);
      translations.push_back(-offset);
    }
  }
  // Corners are important: the recorded state requires coupled XYZ correction.
  if (config_.position_tolerance > 0.0) {
    for (int mask = 0; mask < 8; ++mask) {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      for (int axis = 0; axis < 3; ++axis) {
        offset[axis] = (mask & (1 << axis) ? 1.0 : -1.0) *
          config_.position_tolerance;
      }
      translations.push_back(offset);
    }
  }

  struct Rotation {int axis; double angle;};
  std::vector<Rotation> rotations{{-1, 0.0}};
  for (double angle = config_.orientation_step;
    angle <= config_.orientation_tolerance + 1e-12; angle += config_.orientation_step)
  {
    for (int axis = 0; axis < 3; ++axis) {
      rotations.push_back({axis, angle});
      rotations.push_back({axis, -angle});
    }
  }

  struct ScoredPose {double cost; std::size_t order; Eigen::Isometry3d pose;};
  std::vector<ScoredPose> scored;
  std::size_t order = 0;
  for (const auto & translation : translations) {
    for (const auto & rotation : rotations) {
      Eigen::Isometry3d pose = nominal;
      pose.translation() += translation;
      if (rotation.axis >= 0) {
        pose.linear() = pose.linear() * Eigen::AngleAxisd(
          rotation.angle, Eigen::Vector3d::Unit(rotation.axis)).toRotationMatrix();
      }
      const double cost = translation.norm() /
        std::max(config_.position_tolerance, 1e-9) +
        std::abs(rotation.angle) / std::max(config_.orientation_tolerance, 1e-9);
      int translated_axes = 0;
      for (int axis = 0; axis < 3; ++axis) {
        translated_axes += std::abs(translation[axis]) > 1e-12 ? 1 : 0;
      }
      const bool rotated = rotation.axis >= 0;
      int category = 3;
      if (translated_axes == 0 && !rotated) {
        category = 0;
      } else if ((translated_axes == 1 && !rotated) ||
        (translated_axes == 0 && rotated))
      {
        category = 1;
      } else if (!rotated) {
        category = 2;
      }
      // Reserve the default projection budget for nominal and pure
      // translation/rotation corrections before coupled corrections.
      const double ordering_cost = 100.0 * static_cast<double>(category) + cost;
      scored.push_back({ordering_cost, order++, pose});
    }
  }
  std::stable_sort(scored.begin(), scored.end(), [](const auto & a, const auto & b) {
      return a.cost < b.cost || (a.cost == b.cost && a.order < b.order);
    });
  std::vector<Eigen::Isometry3d> result;
  result.reserve(std::min(config_.projection_limit, scored.size()));
  for (const auto & candidate : scored) {
    result.push_back(candidate.pose);
    if (result.size() == config_.projection_limit) {
      break;
    }
  }
  return result;
}

std::vector<ClosedChainWaypoint> ClosedChainPathPlanner::densify(
  const std::vector<ClosedChainWaypoint> & control_points) const
{
  if (control_points.empty()) {
    return {};
  }
  std::vector<ClosedChainWaypoint> result;
  result.push_back(control_points.front());
  for (std::size_t index = 1; index < control_points.size(); ++index) {
    const auto & from = control_points[index - 1];
    const auto & to = control_points[index];
    const double distance = (to.pose.translation() - from.pose.translation()).norm();
    const double angle = rotationDistance(from.pose, to.pose);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
      distance / config_.validation_position_step,
      angle / config_.validation_orientation_step))));
    for (int step = 1; step <= steps; ++step) {
      ClosedChainWaypoint waypoint = to;
      waypoint.pose = interpolate(from.pose, to.pose, static_cast<double>(step) / steps);
      // Project only interior held-object points; route endpoints remain exact.
      waypoint.allow_projection = to.allow_projection && step < steps;
      result.push_back(std::move(waypoint));
    }
  }
  return result;
}

bool ClosedChainPathPlanner::search(
  const std::vector<double> & start, const std::vector<ClosedChainWaypoint> & waypoints,
  const std::chrono::steady_clock::time_point & deadline, const ClosedChainCancel & canceled,
  const ClosedChainSolve & solve, ClosedChainPath & path, ClosedChainSearchReport & report) const
{
  const auto started = std::chrono::steady_clock::now();
  std::vector<Branch> beam(1);
  beam.front().last = start;
  const auto dense = densify(waypoints);
  if (dense.size() < 2) {
    report.detail = "closed-chain path requires at least two control points";
    report.failure = ClosedChainFailure::IK;
    return false;
  }
  beam.front().last_pose = dense.front().pose;

  for (std::size_t waypoint_index = 1; waypoint_index < dense.size(); ++waypoint_index) {
    report.segment = dense[waypoint_index].segment;
    report.waypoint = waypoint_index;
    if (canceled()) {
      report.failure = ClosedChainFailure::CANCELED;
      report.detail = "closed-chain search canceled";
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      report.failure = ClosedChainFailure::DEADLINE;
      report.detail = "closed-chain search deadline reached";
      return false;
    }
    std::vector<Branch> next;
    const auto projected = dense[waypoint_index].allow_projection ?
      projections(dense[waypoint_index].pose) :
      std::vector<Eigen::Isometry3d>{dense[waypoint_index].pose};
    std::vector<std::size_t> predecessor_solutions(beam.size(), 0U);
    for (const auto & pose : projected) {
      for (std::size_t predecessor_index = 0; predecessor_index < beam.size();
        ++predecessor_index)
      {
        if (predecessor_solutions[predecessor_index] >= config_.solutions_per_branch) {
          continue;
        }
        const auto & predecessor = beam[predecessor_index];
        if (canceled() || std::chrono::steady_clock::now() >= deadline) {
          report.failure = canceled() ? ClosedChainFailure::CANCELED :
            ClosedChainFailure::DEADLINE;
          report.detail = canceled() ? "closed-chain search canceled" :
            "closed-chain search deadline reached";
          return false;
        }
        auto solutions = solve(
          predecessor.last, predecessor.last_pose, pose,
          config_.solutions_per_branch, deadline, canceled, report);
        report.attempts += 1;
        for (auto & solution : solutions) {
          if (solution.failure != ClosedChainFailure::NONE ||
            solution.maximum_joint_step > config_.maximum_joint_step)
          {
            continue;
          }
          if ((solution.realized_pose.translation() -
            predecessor.last_pose.translation()).norm() >
            config_.validation_position_step + 1e-12 ||
            rotationDistance(solution.realized_pose, predecessor.last_pose) >
            config_.validation_orientation_step + 1e-12)
          {
            report.failure = ClosedChainFailure::CONTINUITY;
            report.detail = "realized box-pose edge exceeds dense validation step";
            continue;
          }
          Branch branch = predecessor;
          const double motion = jointDistance(predecessor.last, solution.joints);
          const double correction =
            (solution.realized_pose.translation() - dense[waypoint_index].pose.translation()).norm() +
            0.1 * rotationDistance(solution.realized_pose, dense[waypoint_index].pose);
          std::vector<double> delta(solution.joints.size(), 0.0);
          double smoothness = 0.0;
          for (std::size_t i = 0; i < delta.size() && i < predecessor.last.size(); ++i) {
            delta[i] = solution.joints[i] - predecessor.last[i];
            if (i < predecessor.previous_delta.size()) {
              smoothness += std::abs(delta[i] - predecessor.previous_delta[i]);
            }
          }
          branch.path.score += 5.0 * correction + motion + 0.15 * smoothness -
            0.05 * std::min(solution.joint_margin, 1.0);
          branch.path.states.push_back(std::move(solution));
          branch.path.attempts = report.attempts;
          branch.last = branch.path.states.back().joints;
          branch.last_pose = branch.path.states.back().realized_pose;
          branch.previous_delta = std::move(delta);
          next.push_back(std::move(branch));
          ++predecessor_solutions[predecessor_index];
          if (predecessor_solutions[predecessor_index] >= config_.solutions_per_branch) {
            break;
          }
        }
      }
      // Every retained predecessor gets a chance at the same correction before
      // spending budget on a less-preferred projection. Once a full beam is
      // available, ranking it is more useful than generating redundant
      // corrections from the earliest predecessors.
      if (next.size() >= config_.beam_width) {
        break;
      }
    }
    // A failed layer is repaired locally with a bounded midpoint dog-leg. The
    // original waypoint is still solved afterwards, so no unchecked segment is skipped.
    if (next.empty() && config_.position_tolerance > 0.0) {
      const Eigen::Isometry3d midpoint = interpolate(
        dense[waypoint_index - 1].pose, dense[waypoint_index].pose, 0.5);
      for (const auto & predecessor : beam) {
        for (int axis = -1; axis < 3; ++axis) {
          for (const double sign : {-1.0, 1.0}) {
            if (axis < 0 && sign > 0.0) {
              continue;
            }
            if (canceled() || std::chrono::steady_clock::now() >= deadline) {
              report.failure = canceled() ? ClosedChainFailure::CANCELED :
                ClosedChainFailure::DEADLINE;
              report.detail = canceled() ? "closed-chain search canceled" :
                "closed-chain search deadline reached";
              return false;
            }
            Eigen::Isometry3d detour = midpoint;
            if (axis >= 0) {
              const double half_edge =
                (midpoint.translation() - predecessor.last_pose.translation()).norm();
              const double maximum_offset = std::sqrt(std::max(
                0.0, config_.validation_position_step * config_.validation_position_step -
                half_edge * half_edge));
              detour.translation()[axis] += sign *
                std::min(config_.position_tolerance, maximum_offset);
            }
            auto detour_solutions = solve(
              predecessor.last, predecessor.last_pose, detour, config_.solutions_per_branch,
              deadline, canceled, report);
            ++report.attempts;
            for (auto & detour_solution : detour_solutions) {
              if (detour_solution.failure != ClosedChainFailure::NONE ||
                detour_solution.maximum_joint_step > config_.maximum_joint_step)
              {
                continue;
              }
              if ((detour_solution.realized_pose.translation() -
                predecessor.last_pose.translation()).norm() >
                config_.validation_position_step + 1e-12 ||
                rotationDistance(detour_solution.realized_pose, predecessor.last_pose) >
                config_.validation_orientation_step + 1e-12)
              {
                continue;
              }
              for (const auto & pose : projected) {
                auto repaired = solve(
                  detour_solution.joints, detour_solution.realized_pose, pose,
                  config_.solutions_per_branch,
                  deadline, canceled, report);
                ++report.attempts;
                for (auto & solution : repaired) {
                  if (solution.failure != ClosedChainFailure::NONE ||
                    solution.maximum_joint_step > config_.maximum_joint_step)
                  {
                    continue;
                  }
                  if ((solution.realized_pose.translation() -
                    detour_solution.realized_pose.translation()).norm() >
                    config_.validation_position_step + 1e-12 ||
                    rotationDistance(solution.realized_pose, detour_solution.realized_pose) >
                    config_.validation_orientation_step + 1e-12)
                  {
                    continue;
                  }
                  Branch branch = predecessor;
                  branch.path.score += jointDistance(predecessor.last, detour_solution.joints) +
                    jointDistance(detour_solution.joints, solution.joints) + 1.0;
                  branch.path.states.push_back(detour_solution);
                  branch.path.states.push_back(std::move(solution));
                  branch.path.attempts = report.attempts;
                  branch.last = branch.path.states.back().joints;
                  branch.last_pose = branch.path.states.back().realized_pose;
                  next.push_back(std::move(branch));
                }
              }
            }
          }
        }
      }
    }
    std::stable_sort(next.begin(), next.end(), [](const Branch & a, const Branch & b) {
        return a.path.score < b.path.score;
      });
    std::vector<Branch> diverse;
    for (auto & candidate : next) {
      bool duplicate = false;
      for (const auto & kept : diverse) {
        if (jointDistance(candidate.last, kept.last) < 1e-4) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        diverse.push_back(std::move(candidate));
      }
      if (diverse.size() == config_.beam_width) {
        break;
      }
    }
    if (diverse.empty()) {
      report.failure = report.failure == ClosedChainFailure::NONE ?
        ClosedChainFailure::IK : report.failure;
      report.detail = "no branch reaches waypoint without skipping the segment";
      report.elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      return false;
    }
    beam = std::move(diverse);
  }
  path = std::move(beam.front().path);
  report.failure = ClosedChainFailure::NONE;
  report.elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  return true;
}

}  // namespace agibot_x2_manipulation
