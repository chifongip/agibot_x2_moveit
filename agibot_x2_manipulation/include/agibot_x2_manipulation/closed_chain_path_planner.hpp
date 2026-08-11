#pragma once

#include <Eigen/Geometry>

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{

struct ClosedChainPlannerConfig
{
  std::size_t beam_width{8};
  std::size_t solutions_per_branch{2};
  std::size_t projection_limit{32};
  double position_tolerance{0.010};
  double orientation_tolerance{0.0523598776};
  double position_step{0.005};
  double orientation_step{0.0261799388};
  double validation_position_step{0.005};
  double validation_orientation_step{0.0174533};
  double maximum_joint_step{0.35};
};

enum class ClosedChainFailure
{
  NONE,
  CANCELED,
  DEADLINE,
  IK,
  BOUNDS,
  COLLISION,
  CONTINUITY,
  CONTACT
};

const char * closedChainFailureName(ClosedChainFailure failure);

struct ClosedChainSolution
{
  std::vector<double> joints;
  Eigen::Isometry3d realized_pose{Eigen::Isometry3d::Identity()};
  double joint_margin{0.0};
  double maximum_joint_step{0.0};
  ClosedChainFailure failure{ClosedChainFailure::NONE};
  std::string failed_arm;
  std::string ik_mode;
};

struct ClosedChainWaypoint
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  bool allow_projection{true};
  std::string segment;
};

enum class ClosedChainRoute
{
  DIRECT,
  LOW_XY_THEN_LIFT,
  LIFT_THEN_XY,
  ROTATE_BEFORE_TRANSLATION,
  ROTATE_AFTER_TRANSLATION,
  DOGLEG_NEGATIVE_Y,
  DOGLEG_POSITIVE_Y
};

const char * closedChainRouteName(ClosedChainRoute route);

std::vector<ClosedChainWaypoint> makePlaceRouteWaypoints(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & place,
  double lift_height, double dogleg_y, bool from_pick, ClosedChainRoute route);

struct ClosedChainPath
{
  std::vector<ClosedChainSolution> states;
  double score{0.0};
  std::size_t attempts{0};
};

struct ClosedChainSearchReport
{
  ClosedChainFailure failure{ClosedChainFailure::NONE};
  std::string segment;
  std::size_t waypoint{0};
  std::size_t attempts{0};
  double elapsed{0.0};
  std::string detail;
  std::string failed_arm;
  std::string ik_mode;
};

using ClosedChainCancel = std::function<bool()>;
using ClosedChainSolve = std::function<std::vector<ClosedChainSolution>(
    const std::vector<double> &, const Eigen::Isometry3d &, const Eigen::Isometry3d &,
    std::size_t,
    const std::chrono::steady_clock::time_point &, const ClosedChainCancel &,
    ClosedChainSearchReport &)>;

class ClosedChainPathPlanner
{
public:
  explicit ClosedChainPathPlanner(ClosedChainPlannerConfig config);

  const ClosedChainPlannerConfig & config() const {return config_;}

  std::vector<Eigen::Isometry3d> projections(const Eigen::Isometry3d & nominal) const;

  std::vector<ClosedChainWaypoint> densify(
    const std::vector<ClosedChainWaypoint> & control_points) const;

  bool search(
    const std::vector<double> & start, const std::vector<ClosedChainWaypoint> & waypoints,
    const std::chrono::steady_clock::time_point & deadline, const ClosedChainCancel & canceled,
    const ClosedChainSolve & solve, ClosedChainPath & path, ClosedChainSearchReport & report) const;

private:
  ClosedChainPlannerConfig config_;
};

}  // namespace agibot_x2_manipulation
