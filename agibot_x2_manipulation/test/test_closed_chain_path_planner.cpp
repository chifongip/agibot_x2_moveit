#include "agibot_x2_manipulation/closed_chain_path_planner.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d pose(double x)
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation().x() = x;
  return value;
}

ClosedChainSolution solution(double joint, const Eigen::Isometry3d & realized)
{
  ClosedChainSolution result;
  result.joints = {joint};
  result.realized_pose = realized;
  result.joint_margin = 0.5;
  result.maximum_joint_step = 0.2;
  return result;
}

TEST(ClosedChainPathPlanner, BeamRetainsBranchThatGreedySearchLoses)
{
  ClosedChainPlannerConfig config;
  config.beam_width = 8;
  config.solutions_per_branch = 2;
  config.position_tolerance = 0.0;
  config.validation_position_step = 1.0;
  ClosedChainPathPlanner planner(config);
  const std::vector<ClosedChainWaypoint> waypoints{
    {pose(0.0), false, "approach"},
    {pose(0.5), false, "approach"},
    {pose(1.0), false, "approach"}};
  const ClosedChainSolve solve = [](
    const std::vector<double> & seed, const Eigen::Isometry3d &,
    const Eigen::Isometry3d & target, std::size_t,
    const auto &, const auto &, ClosedChainSearchReport &) {
      if (target.translation().x() < 0.75) {
        return std::vector<ClosedChainSolution>{solution(1.0, target), solution(2.0, target)};
      }
      if (std::abs(seed.front() - 2.0) < 1e-9) {
        return std::vector<ClosedChainSolution>{solution(3.0, target)};
      }
      return std::vector<ClosedChainSolution>{};
    };
  ClosedChainPath path;
  ClosedChainSearchReport report;
  ASSERT_TRUE(planner.search(
    {0.0}, waypoints, std::chrono::steady_clock::now() + std::chrono::seconds(1),
    []() {return false;}, solve, path, report));
  ASSERT_EQ(path.states.size(), 2U);
  EXPECT_DOUBLE_EQ(path.states.front().joints.front(), 2.0);
  EXPECT_DOUBLE_EQ(path.states.back().joints.front(), 3.0);

  config.beam_width = 1;
  ClosedChainPathPlanner greedy(config);
  EXPECT_FALSE(greedy.search(
    {0.0}, waypoints, std::chrono::steady_clock::now() + std::chrono::seconds(1),
    []() {return false;}, solve, path, report));
}

TEST(ClosedChainPathPlanner, ProjectionSetContainsCombinedTranslationAndRotation)
{
  ClosedChainPlannerConfig config;
  config.projection_limit = 128;
  config.position_tolerance = 0.01;
  config.position_step = 0.01;
  config.orientation_tolerance = 0.0523598776;
  config.orientation_step = 0.0523598776;
  ClosedChainPathPlanner planner(config);
  bool found_combined = false;
  for (const auto & candidate : planner.projections(Eigen::Isometry3d::Identity())) {
    const double angle = Eigen::AngleAxisd(candidate.linear()).angle();
    int translated_axes = 0;
    for (int axis = 0; axis < 3; ++axis) {
      translated_axes += std::abs(candidate.translation()[axis]) > 1e-9 ? 1 : 0;
    }
    found_combined = found_combined || (translated_axes >= 2 && angle > 1e-9);
  }
  EXPECT_TRUE(found_combined);
}

TEST(ClosedChainPathPlanner, DefaultProjectionBudgetRetainsPureAndCoupledCorrections)
{
  ClosedChainPlannerConfig config;
  ClosedChainPathPlanner planner(config);
  bool pure_translation = false;
  bool pure_rotation = false;
  bool coupled_xyz = false;
  for (const auto & candidate : planner.projections(Eigen::Isometry3d::Identity())) {
    const double angle = Eigen::AngleAxisd(candidate.linear()).angle();
    int translated_axes = 0;
    for (int axis = 0; axis < 3; ++axis) {
      translated_axes += std::abs(candidate.translation()[axis]) > 1e-9 ? 1 : 0;
    }
    pure_translation = pure_translation || (translated_axes == 1 && angle < 1e-9);
    pure_rotation = pure_rotation || (translated_axes == 0 && angle > 1e-9);
    coupled_xyz = coupled_xyz || translated_axes == 3;
  }
  EXPECT_TRUE(pure_translation);
  EXPECT_TRUE(pure_rotation);
  EXPECT_TRUE(coupled_xyz);
}

TEST(ClosedChainPathPlanner, BeamExpandsThirdRetainedPredecessor)
{
  ClosedChainPlannerConfig config;
  config.beam_width = 3;
  config.solutions_per_branch = 3;
  config.position_tolerance = 0.0;
  config.validation_position_step = 1.0;
  ClosedChainPathPlanner planner(config);
  const ClosedChainSolve solve = [](
    const std::vector<double> & seed, const Eigen::Isometry3d &,
    const Eigen::Isometry3d & target, std::size_t,
    const auto &, const auto &, ClosedChainSearchReport &) {
      if (target.translation().x() < 0.75) {
        return std::vector<ClosedChainSolution>{
          solution(1.0, target), solution(2.0, target), solution(3.0, target)};
      }
      if (std::abs(seed.front() - 3.0) < 1e-9) {
        return std::vector<ClosedChainSolution>{solution(4.0, target)};
      }
      return std::vector<ClosedChainSolution>{};
    };
  ClosedChainPath path;
  ClosedChainSearchReport report;
  ASSERT_TRUE(planner.search(
    {0.0}, {{pose(0.0), false, "carry"}, {pose(0.5), false, "carry"},
      {pose(1.0), false, "carry"}},
    std::chrono::steady_clock::now() + std::chrono::seconds(1),
    []() {return false;}, solve, path, report));
  EXPECT_DOUBLE_EQ(path.states.front().joints.front(), 3.0);
  EXPECT_DOUBLE_EQ(path.states.back().joints.front(), 4.0);
}

TEST(ClosedChainPathPlanner, FailedEdgeIsRepairedThroughExactMidpoint)
{
  ClosedChainPlannerConfig config;
  config.position_tolerance = 0.01;
  config.projection_limit = 1;
  config.validation_position_step = 2.0;
  ClosedChainPathPlanner planner(config);
  const ClosedChainSolve solve = [](
    const std::vector<double> & seed, const Eigen::Isometry3d &,
    const Eigen::Isometry3d & target, std::size_t,
    const auto &, const auto &, ClosedChainSearchReport &) {
      const double x = target.translation().x();
      if (std::abs(x - 0.5) < 1e-9 && std::abs(seed.front()) < 1e-9) {
        return std::vector<ClosedChainSolution>{solution(0.5, target)};
      }
      if (std::abs(x - 1.0) < 1e-9 && std::abs(seed.front() - 0.5) < 1e-9) {
        return std::vector<ClosedChainSolution>{solution(1.0, target)};
      }
      return std::vector<ClosedChainSolution>{};
    };
  ClosedChainPath path;
  ClosedChainSearchReport report;
  ASSERT_TRUE(planner.search(
    {0.0}, {{pose(0.0), false, "carry"}, {pose(1.0), false, "carry"}},
    std::chrono::steady_clock::now() + std::chrono::seconds(1),
    []() {return false;}, solve, path, report));
  ASSERT_EQ(path.states.size(), 2U);
  EXPECT_DOUBLE_EQ(path.states.front().realized_pose.translation().x(), 0.5);
}

TEST(ClosedChainPathPlanner, DensificationAndCancellationAreStrict)
{
  ClosedChainPlannerConfig config;
  config.validation_position_step = 0.005;
  config.validation_orientation_step = M_PI / 180.0;
  ClosedChainPathPlanner planner(config);
  auto end = pose(0.011);
  end.linear() = Eigen::AngleAxisd(2.1 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
    .toRotationMatrix();
  const auto dense = planner.densify({{pose(0.0), false, "carry"}, {end, true, "carry"}});
  ASSERT_EQ(dense.size(), 4U);
  for (std::size_t index = 1; index < dense.size(); ++index) {
    EXPECT_LE(
      (dense[index].pose.translation() - dense[index - 1].pose.translation()).norm(),
      0.005 + 1e-12);
  }
  ClosedChainPath path;
  ClosedChainSearchReport report;
  std::atomic<bool> called{false};
  EXPECT_FALSE(planner.search(
    {0.0}, {{pose(0.0), false, "carry"}, {pose(0.01), false, "carry"}},
    std::chrono::steady_clock::now() + std::chrono::seconds(1),
    [&called]() {called = true; return true;},
    [](const auto &, const auto &, const auto &, auto, const auto &, const auto &, auto &) {
      return std::vector<ClosedChainSolution>{};
    }, path, report));
  EXPECT_TRUE(called.load());
  EXPECT_EQ(report.failure, ClosedChainFailure::CANCELED);
}

TEST(ClosedChainPathPlanner, ExpiredDeadlineDoesNotInvokeIk)
{
  ClosedChainPathPlanner planner(ClosedChainPlannerConfig{});
  ClosedChainPath path;
  ClosedChainSearchReport report;
  int calls = 0;
  EXPECT_FALSE(planner.search(
    {0.0}, {{pose(0.0), false, "carry"}, {pose(0.01), false, "carry"}},
    std::chrono::steady_clock::now() - std::chrono::milliseconds(1),
    []() {return false;},
    [&calls](const auto &, const auto &, const auto &, auto, const auto &, const auto &, auto &) {
      ++calls;
      return std::vector<ClosedChainSolution>{};
    }, path, report));
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(report.failure, ClosedChainFailure::DEADLINE);
}

TEST(ClosedChainPathPlanner, PlaceRoutesPreserveExactEndpointsAndSegmentOrder)
{
  Eigen::Isometry3d from = Eigen::Isometry3d::Identity();
  from.translation() = Eigen::Vector3d(0.30, -0.05, 0.20);
  Eigen::Isometry3d place = Eigen::Isometry3d::Identity();
  place.translation() = Eigen::Vector3d(0.42, 0.08, 0.28);
  place.linear() = Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto direct = makePlaceRouteWaypoints(
    from, place, 0.05, 0.03, true, ClosedChainRoute::DIRECT);
  ASSERT_EQ(direct.size(), 4U);
  EXPECT_EQ(direct[1].segment, "pick_lift");
  EXPECT_EQ(direct[2].segment, "move_to_place_lift");
  EXPECT_EQ(direct.back().segment, "place_descent");
  EXPECT_TRUE(direct.front().pose.matrix().isApprox(from.matrix()));
  EXPECT_TRUE(direct.back().pose.matrix().isApprox(place.matrix()));

  const auto standalone = makePlaceRouteWaypoints(
    from, place, 0.05, 0.03, false, ClosedChainRoute::DIRECT);
  ASSERT_EQ(standalone.size(), 3U);
  EXPECT_EQ(standalone[1].segment, "move_to_place_lift");

  const auto rotate_first = makePlaceRouteWaypoints(
    from, place, 0.05, 0.03, true, ClosedChainRoute::ROTATE_BEFORE_TRANSLATION);
  ASSERT_EQ(rotate_first.size(), 5U);
  EXPECT_EQ(rotate_first[2].segment, "rotate_before_translation");
  EXPECT_TRUE(rotate_first[2].pose.linear().isApprox(place.linear()));

  const auto rotate_last = makePlaceRouteWaypoints(
    from, place, 0.05, 0.03, true, ClosedChainRoute::ROTATE_AFTER_TRANSLATION);
  ASSERT_EQ(rotate_last.size(), 5U);
  EXPECT_EQ(rotate_last[2].segment, "place_translation");
  EXPECT_TRUE(rotate_last[2].pose.linear().isApprox(from.linear()));
  EXPECT_EQ(rotate_last[3].segment, "rotate_after_translation");
}

TEST(ClosedChainPathPlanner, PlaceDoglegsAreBoundedAndDeterministic)
{
  Eigen::Isometry3d from = Eigen::Isometry3d::Identity();
  from.translation() = Eigen::Vector3d(0.30, -0.05, 0.20);
  Eigen::Isometry3d place = Eigen::Isometry3d::Identity();
  place.translation() = Eigen::Vector3d(0.42, 0.08, 0.28);
  constexpr double lift = 0.05;
  constexpr double dogleg = 0.03;
  const auto negative = makePlaceRouteWaypoints(
    from, place, lift, dogleg, false, ClosedChainRoute::DOGLEG_NEGATIVE_Y);
  const auto positive = makePlaceRouteWaypoints(
    from, place, lift, dogleg, false, ClosedChainRoute::DOGLEG_POSITIVE_Y);
  ASSERT_EQ(negative.size(), 4U);
  ASSERT_EQ(positive.size(), 4U);
  const double nominal_midpoint_y = 0.5 * (from.translation().y() + place.translation().y());
  EXPECT_NEAR(negative[1].pose.translation().y(), nominal_midpoint_y - dogleg, 1e-12);
  EXPECT_NEAR(positive[1].pose.translation().y(), nominal_midpoint_y + dogleg, 1e-12);
  EXPECT_TRUE(negative.back().pose.matrix().isApprox(place.matrix()));
  EXPECT_TRUE(positive.back().pose.matrix().isApprox(place.matrix()));
  EXPECT_THROW(
    makePlaceRouteWaypoints(
      from, place, -lift, dogleg, false, ClosedChainRoute::DIRECT),
    std::invalid_argument);
}

}  // namespace
}  // namespace agibot_x2_manipulation
