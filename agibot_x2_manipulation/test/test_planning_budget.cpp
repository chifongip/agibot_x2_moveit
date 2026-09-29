#include "agibot_x2_manipulation/planning_budget.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace agibot_x2_manipulation
{
namespace
{

TEST(PlanningBudget, FastAttemptReservesFallbackAndCapsNominalWork)
{
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(30.0, 8.0), 6.0);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(80.0, 3.25), 3.25);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(1.0, 8.0), 0.2);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(0.0, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(-1.0, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(30.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(std::numeric_limits<double>::infinity(), 8.0), 0.0);
  EXPECT_DOUBLE_EQ(fastAttemptTimeout(30.0, std::numeric_limits<double>::quiet_NaN()), 0.0);
}

TEST(PlanningBudget, ExpiredFastAttemptRestoresFallbackDeadline)
{
  using Clock = std::chrono::steady_clock;
  const auto phase = Clock::time_point(std::chrono::seconds(30));
  const auto fast = Clock::time_point(std::chrono::seconds(6));
  const auto now = Clock::time_point(std::chrono::seconds(7));
  auto active = phase;
  bool action_canceled = false;
  const auto interrupted = [&]() {return action_canceled || now >= active;};
  {
    ScopedPlanningDeadline scope(active, fast);
    EXPECT_TRUE(interrupted());
    {
      ScopedPlanningDeadline nested(active, phase);
      EXPECT_EQ(active, fast);
    }
  }
  EXPECT_EQ(active, phase);
  EXPECT_FALSE(interrupted());
  try {
    ScopedPlanningDeadline scope(active, fast);
    throw 1;
  } catch (int) {}
  EXPECT_EQ(active, phase);
  action_canceled = true;
  EXPECT_TRUE(interrupted());
}

TEST(PlanningBudget, SharesRemainingBudgetAcrossRetries)
{
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 10.0, 3U), 20.0 / 3.0);
}

TEST(PlanningBudget, CarriesUnusedTimeToLaterRetries)
{
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 13.0, 2U), 8.5);
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 18.0, 1U), 12.0);
}

TEST(PlanningBudget, RejectsExhaustedOrInvalidBudgets)
{
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 30.0, 1U), 0.0);
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 31.0, 1U), 0.0);
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(0.0, 0.0, 1U), 0.0);
  EXPECT_DOUBLE_EQ(adaptiveRetryTimeout(30.0, 0.0, 0U), 0.0);
}

TEST(PlanningBudget, EndpointRouteIncludesEveryPlanAndSceneRoundTrips)
{
  EXPECT_DOUBLE_EQ(endpointRouteTimeout(30.0, 2.0, 3U), 8.0);
  EXPECT_DOUBLE_EQ(endpointRouteTimeout(1.0, 2.0, 3U), 1.0);
  EXPECT_DOUBLE_EQ(endpointRouteTimeout(0.0, 2.0, 3U), 0.0);
  EXPECT_DOUBLE_EQ(endpointRouteTimeout(30.0, 2.0, 0U), 0.0);
}

}  // namespace
}  // namespace agibot_x2_manipulation
