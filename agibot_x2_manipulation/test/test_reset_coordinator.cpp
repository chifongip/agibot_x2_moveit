#include "agibot_x2_manipulation/reset_coordinator.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>

namespace agibot_x2_manipulation
{
namespace
{

using namespace std::chrono_literals;

TEST(ResetCoordinator, SuccessfulResetBlocksThenUnlocksNormalOperations)
{
  ResetCoordinator coordinator;
  ASSERT_TRUE(coordinator.requestReset());
  EXPECT_TRUE(coordinator.resetRequested());
  EXPECT_FALSE(coordinator.reserveOperation());
  EXPECT_EQ(
    coordinator.waitForResetAccess(10ms, []() {return false;}),
    ResetAcquireResult::ACQUIRED);
  coordinator.finishReset(true);
  EXPECT_FALSE(coordinator.resetRequested());
  EXPECT_TRUE(coordinator.reserveOperation());
  coordinator.releaseOperation();
}

TEST(ResetCoordinator, TimeoutLeavesResetLatchedForRetry)
{
  ResetCoordinator coordinator;
  ASSERT_TRUE(coordinator.reserveOperation());
  ASSERT_TRUE(coordinator.requestReset());
  EXPECT_EQ(
    coordinator.waitForResetAccess(1ms, []() {return false;}),
    ResetAcquireResult::TIMEOUT);
  coordinator.finishReset(false);
  EXPECT_TRUE(coordinator.resetRequested());
  EXPECT_FALSE(coordinator.reserveOperation());
  coordinator.releaseOperation();
  EXPECT_TRUE(coordinator.requestReset());
  EXPECT_EQ(
    coordinator.waitForResetAccess(10ms, []() {return false;}),
    ResetAcquireResult::ACQUIRED);
  coordinator.finishReset(true);
}

TEST(ResetCoordinator, CancellationDoesNotClearSafetyLatch)
{
  ResetCoordinator coordinator;
  ASSERT_TRUE(coordinator.requestReset());
  EXPECT_EQ(
    coordinator.waitForResetAccess(10ms, []() {return true;}),
    ResetAcquireResult::CANCELED);
  coordinator.finishReset(false);
  EXPECT_TRUE(coordinator.resetRequested());
}

TEST(ResetCoordinator, ResetWaitsForActiveOperationAndOwnsAdmissionBeforeResult)
{
  ResetCoordinator coordinator;
  ASSERT_TRUE(coordinator.reserveOperation());
  ASSERT_TRUE(coordinator.requestReset());
  EXPECT_FALSE(coordinator.reserveOperation());
  EXPECT_FALSE(coordinator.requestReset());

  auto acquire = std::async(std::launch::async, [&coordinator]() {
      return coordinator.waitForResetAccess(100ms, []() {return false;});
    });
  coordinator.releaseOperation();
  EXPECT_EQ(acquire.get(), ResetAcquireResult::ACQUIRED);
  EXPECT_FALSE(coordinator.reserveOperation());

  coordinator.finishReset(true);
  EXPECT_TRUE(coordinator.reserveOperation());
  coordinator.releaseOperation();
}

}  // namespace
}  // namespace agibot_x2_manipulation
