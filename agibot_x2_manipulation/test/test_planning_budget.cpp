#include "agibot_x2_manipulation/planning_budget.hpp"

#include <gtest/gtest.h>

namespace agibot_x2_manipulation
{
namespace
{

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
