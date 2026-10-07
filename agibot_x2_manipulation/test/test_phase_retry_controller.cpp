#include "agibot_x2_manipulation/phase_retry_controller.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

namespace agibot_x2_manipulation
{

TEST(PhaseRetryController, OnlyAcceptedContinuePublishesMetadataBeforeTheNextAttempt)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "not_attached");
  int attempts = 0;
  int accepted = 0;
  std::string error;
  EXPECT_TRUE(controller.run("approach", false, 1, 1.0, 0.0,
    [&](auto, std::string & failure) {
      ++attempts;
      if (attempts == 1) {failure = "planning failed"; return false;}
      EXPECT_EQ(accepted, 1);
      return true;
    }, []() {return false;}, [&](const auto & status) {
      if (status.status != "paused") {return;}
      std::string detail;
      const auto metadata = [&]() {++accepted;};
      EXPECT_FALSE(controller.requestContinue("old-task", status.pause_id, detail, metadata));
      EXPECT_FALSE(controller.requestContinue("task", status.pause_id + 1, detail, metadata));
      EXPECT_EQ(accepted, 0);
      EXPECT_TRUE(controller.requestContinue("task", status.pause_id, detail, metadata));
      EXPECT_FALSE(controller.requestContinue("task", status.pause_id, detail, metadata));
      EXPECT_EQ(accepted, 1);
    }, error));
  EXPECT_EQ(attempts, 2);
}

TEST(PhaseRetryController, DeferredMotionDoesNotAdvanceCompletedCheckpoint)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "not_attached");
  controller.checkpoint("saved/pregrasp", "not_attached");
  std::string error;
  EXPECT_TRUE(controller.run("saved/approach", false, 1, 1.0, 0.0,
    [](auto, std::string &) {return true;}, []() {return false;},
    [](const auto &) {}, error, []() {return false;}, []() {return false;}));
  EXPECT_EQ(controller.snapshot().last_completed_phase, "saved/pregrasp");
  EXPECT_EQ(controller.snapshot().object_disposition, "not_attached");
  EXPECT_FALSE(controller.snapshot().can_continue);
}

TEST(PhaseRetryController, FirstFailureRetriesWithoutPausing)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "not_attached");
  int calls = 0;
  std::vector<std::string> states;
  std::string error;
  EXPECT_TRUE(controller.run("approach", false, 3, 1.0, 0.0,
    [&](auto, std::string & failure) {failure = "no route"; return ++calls == 2;},
    []() {return false;}, [&](const auto & status) {states.push_back(status.status);}, error));
  EXPECT_EQ(calls, 2);
  EXPECT_EQ(controller.snapshot().last_completed_phase, "approach");
  EXPECT_EQ(std::count(states.begin(), states.end(), "paused"), 0);
}

TEST(PhaseRetryController, PlanOnlyReturnsFailureWithoutWaitingForContinue)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "not_attached");
  int calls = 0;
  std::string error;
  EXPECT_FALSE(controller.run("planning", true, 3, 1.0, 0.0,
    [&](auto, std::string & failure) {++calls; failure = "IK failed"; return false;},
    []() {return false;}, [](const auto &) {}, error));
  EXPECT_EQ(calls, 3);
  EXPECT_EQ(error, "IK failed");
  EXPECT_FALSE(controller.snapshot().can_continue);
}

TEST(PhaseRetryController, ContinueResumesOnlyUnfinishedPhaseAndRejectsDuplicateRequests)
{
  PhaseRetryController controller;
  controller.begin("task", "pick_place", "attached");
  controller.checkpoint("attach", "attached");
  int calls = 0;
  std::string error;
  std::string request_error;
  EXPECT_TRUE(controller.run("carry", false, 2, 1.0, 0.0,
    [&](auto, std::string & failure) {failure = "no route"; return ++calls == 3;},
    []() {return false;}, [&](const auto & status) {
      if (status.status == "paused") {
        EXPECT_EQ(status.last_completed_phase, "attach");
        EXPECT_EQ(status.object_disposition, "attached");
        EXPECT_FALSE(controller.requestContinue("other", status.pause_id, request_error));
        EXPECT_FALSE(controller.requestContinue("task", status.pause_id - 1, request_error));
        EXPECT_TRUE(controller.requestContinue("task", status.pause_id, request_error));
        EXPECT_FALSE(controller.requestContinue("task", status.pause_id, request_error));
      }
    }, error));
  EXPECT_EQ(calls, 3);
  EXPECT_EQ(controller.snapshot().last_completed_phase, "carry");
  EXPECT_EQ(controller.snapshot().pause_id, 1U);
}

TEST(PhaseRetryController, ReleaseCheckpointRemainsReleasedDuringReturnRetries)
{
  PhaseRetryController controller;
  controller.begin("task", "place", "attached");
  controller.checkpoint("release", "released");
  int calls = 0;
  std::string error;
  EXPECT_TRUE(controller.run("retreat", false, 2, 1.0, 0.0,
    [&](auto, std::string &) {return ++calls == 2;}, []() {return false;},
    [&](const auto & status) {EXPECT_EQ(status.object_disposition, "released");}, error));
  EXPECT_EQ(controller.snapshot().object_disposition, "released");
}

TEST(PhaseRetryController, CancellationWakesPausedWorkerAndPreservesHeldObject)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "attached");
  std::atomic<bool> canceled{false};
  std::atomic<bool> paused{false};
  bool result = true;
  std::string error;
  std::thread worker([&]() {
    result = controller.run("carry", false, 1, 1.0, 0.0,
      [](auto, std::string &) {return false;}, [&]() {return canceled.load();},
      [&](const auto & status) {if (status.status == "paused") {paused = true;}}, error);
  });
  const auto deadline = PhaseRetryController::Clock::now() + std::chrono::seconds(1);
  while (!paused && PhaseRetryController::Clock::now() < deadline) {std::this_thread::yield();}
  EXPECT_TRUE(paused);
  canceled = true;
  worker.join();
  EXPECT_FALSE(result);
  EXPECT_EQ(controller.snapshot().status, "canceled");
  EXPECT_EQ(controller.snapshot().object_disposition, "attached");
  EXPECT_FALSE(controller.snapshot().can_continue);
}

TEST(PhaseRetryController, RetryBudgetIsSharedAndDoesNotInterruptSuccessfulExecution)
{
  PhaseRetryController controller;
  controller.begin("task", "pick", "not_attached");
  std::vector<PhaseRetryController::Clock::time_point> deadlines;
  int calls = 0;
  std::string error;
  EXPECT_TRUE(controller.run("approach", false, 3, 0.01, 0.0,
    [&](auto deadline, std::string &) {
      deadlines.push_back(deadline);
      if (++calls == 1) {return false;}
      // Successful execution may finish after the planning budget.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return true;
    }, []() {return false;}, [](const auto &) {}, error));
  ASSERT_EQ(deadlines.size(), 2U);
  EXPECT_EQ(deadlines[0], deadlines[1]);
}

TEST(PhaseRetryController, AmbiguousPhysicalOperationDoesNotDispatchAgain)
{
  for (const auto & phase : {"attach", "release"}) {
    SCOPED_TRACE(phase);
    PhaseRetryController controller;
    controller.begin("task", "pick_place", "uncertain");
    int calls = 0;
    std::string error;
    EXPECT_FALSE(controller.run(phase, false, 3, 1.0, 0.0,
      [&](auto, std::string & failure) {++calls; failure = "physical outcome unknown"; return false;},
      []() {return false;}, [](const auto &) {}, error, []() {return true;}));
    EXPECT_EQ(calls, 1);
    EXPECT_FALSE(controller.snapshot().can_continue);
  }
}

TEST(PhaseRetryController, EveryActionPreservesCheckpointsAcrossRepeatedContinueCycles)
{
  for (const auto & action : {"pick", "place", "pick_place", "move_carry_pose", "reset"}) {
    SCOPED_TRACE(action);
    PhaseRetryController controller;
    controller.begin("task", action, "attached");
    controller.checkpoint("physical_checkpoint", "attached");
    int calls = 0;
    unsigned int last_pause = 0;
    std::vector<PhaseRetryController::Clock::time_point> deadlines;
    std::string error, request_error;
    ASSERT_TRUE(controller.run("unfinished_motion", false, 1, 1.0, 0.0,
      [&](auto deadline, std::string & failure) {
        deadlines.push_back(deadline);
        failure = "transient planning failure";
        return ++calls == 3;
      }, []() {return false;}, [&](const auto & status) {
        if (status.status != "paused") {return;}
        EXPECT_EQ(status.last_completed_phase, "physical_checkpoint");
        EXPECT_EQ(status.object_disposition, "attached");
        EXPECT_GT(status.pause_id, last_pause);
        EXPECT_FALSE(controller.requestContinue("task", last_pause, request_error));
        last_pause = status.pause_id;
        EXPECT_TRUE(controller.requestContinue("task", status.pause_id, request_error));
        EXPECT_FALSE(controller.requestContinue("task", status.pause_id, request_error));
      }, error));
    ASSERT_EQ(deadlines.size(), 3U);
    EXPECT_GT(deadlines[1], deadlines[0]);
    EXPECT_GT(deadlines[2], deadlines[1]);
    EXPECT_EQ(last_pause, 2U);
    EXPECT_EQ(controller.snapshot().last_completed_phase, "unfinished_motion");
    EXPECT_FALSE(controller.snapshot().can_continue);
    controller.finish("completed");
    EXPECT_FALSE(controller.requestContinue("task", last_pause, request_error));
  }
}

TEST(PhaseRetryController, CancellationAfterContinueDoesNotDispatchAnotherAttempt)
{
  PhaseRetryController controller;
  controller.begin("task", "pick_place", "attached");
  controller.checkpoint("attach", "attached");
  bool canceled = false;
  int calls = 0;
  std::string error, request_error;
  EXPECT_FALSE(controller.run("carry", false, 1, 1.0, 0.0,
    [&](auto, std::string &) {++calls; return false;}, [&]() {return canceled;},
    [&](const auto & status) {
      if (status.status == "paused") {
        EXPECT_TRUE(controller.requestContinue("task", status.pause_id, request_error));
        canceled = true;
      }
    }, error));
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(controller.snapshot().status, "canceled");
  EXPECT_EQ(controller.snapshot().last_completed_phase, "attach");
  EXPECT_EQ(controller.snapshot().object_disposition, "attached");
  EXPECT_FALSE(controller.snapshot().can_continue);
}

}  // namespace agibot_x2_manipulation
