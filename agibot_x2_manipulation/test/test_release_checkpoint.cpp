#include "pick_place/release_checkpoint.hpp"
#include "agibot_x2_manipulation/phase_retry_controller.hpp"

#include <gtest/gtest.h>
#include <thread>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

TEST(ReleaseCheckpoint, InvalidFeedbackDoesNotRequestCorrection)
{
  ReleaseCheckpoint checkpoint;
  int refreshes = 0, checks = 0, corrections = 0;
  std::string error;
  const auto refresh = [&](auto & failure) {
      if (++refreshes == 1) {failure = "held-state collision"; return false;}
      return true;
    };
  const auto check = [&](auto &) {++checks; return true;};
  const auto correct = [&](auto &) {++corrections; return true;};
  EXPECT_FALSE(checkpoint.validate(refresh, check, correct, [] {return false;}, error));
  EXPECT_EQ(checks, 0);
  EXPECT_EQ(corrections, 0);
  EXPECT_TRUE(checkpoint.validate(refresh, check, correct, [] {return false;}, error));
  EXPECT_EQ(checks, 1);
  EXPECT_EQ(corrections, 0);
}

TEST(ReleaseCheckpoint, FeedbackFailureAfterCorrectionDoesNotRepeatMotion)
{
  ReleaseCheckpoint checkpoint;
  int refreshes = 0, checks = 0, corrections = 0;
  std::string error;
  const auto refresh = [&](auto & failure) {
      if (++refreshes == 2) {failure = "feedback unavailable"; return false;}
      return true;
    };
  const auto check = [&](auto &) {return ++checks > 1;};
  const auto correct = [&](auto &) {++corrections; return true;};
  EXPECT_FALSE(checkpoint.validate(refresh, check, correct, [] {return false;}, error));
  EXPECT_EQ(checks, 1);
  EXPECT_EQ(corrections, 1);
  EXPECT_TRUE(checkpoint.validate(refresh, check, correct, [] {return false;}, error));
  EXPECT_EQ(refreshes, 3);
  EXPECT_EQ(checks, 2);
  EXPECT_EQ(corrections, 1);
}

TEST(ReleaseCheckpoint, ValidMeasuredExitNeedsNoCorrection)
{
  ReleaseCheckpoint checkpoint;
  int corrections = 0;
  std::string error;
  EXPECT_TRUE(checkpoint.validate([](auto &) {return true;}, [](auto &) {return true;},
    [&](auto &) {++corrections; return true;}, [] {return false;}, error));
  EXPECT_EQ(corrections, 0);
}

TEST(ReleaseCheckpoint, CorrectionMustPassANewMeasuredCheck)
{
  ReleaseCheckpoint checkpoint;
  std::vector<std::string> calls;
  std::string error;
  const auto check = [&](auto & failure) {
      calls.push_back("check");
      failure = "measured exit remains infeasible";
      return false;
    };
  const auto correct = [&](auto &) {calls.push_back("correct"); return true;};
  EXPECT_FALSE(checkpoint.validate([](auto &) {return true;}, check, correct, [] {return false;}, error));
  EXPECT_EQ(calls, (std::vector<std::string>{"check", "correct", "check"}));
  calls.clear();
  EXPECT_TRUE(checkpoint.validate([](auto &) {return true;}, [&](auto &) {calls.push_back("check"); return true;},
    correct, [] {return false;}, error));
  EXPECT_EQ(calls, (std::vector<std::string>{"correct", "check"}));
}

TEST(ReleaseCheckpoint, FailedCorrectionRemainsPendingOnRetry)
{
  ReleaseCheckpoint checkpoint;
  int checks = 0, corrections = 0;
  std::string error;
  const auto check = [&](auto &) {return ++checks > 1;};
  const auto correct = [&](auto &) {return ++corrections > 1;};
  EXPECT_FALSE(checkpoint.validate([](auto &) {return true;}, check, correct, [] {return false;}, error));
  EXPECT_TRUE(checkpoint.validate([](auto &) {return true;}, check, correct, [] {return false;}, error));
  EXPECT_EQ(checks, 2);
  EXPECT_EQ(corrections, 2);
}

TEST(ReleaseCheckpoint, CorrectionFinishingAtDeadlineIsCheckedWithFreshBudget)
{
  ReleaseCheckpoint checkpoint;
  int checks = 0, corrections = 0;
  bool expired = false;
  std::string error;
  const auto check = [&](auto &) {return ++checks > 1;};
  const auto correct = [&](auto &) {++corrections; expired = true; return true;};
  EXPECT_FALSE(checkpoint.validate([](auto &) {return true;}, check, correct, [&] {return expired;}, error));
  expired = false;
  EXPECT_TRUE(checkpoint.validate([](auto &) {return true;}, check, correct, [&] {return expired;}, error));
  EXPECT_EQ(checks, 2);
  EXPECT_EQ(corrections, 1);
}

TEST(ReleaseCheckpoint, CancellationNeverAuthorizesRelease)
{
  ReleaseCheckpoint checkpoint;
  bool canceled = true;
  int checks = 0, corrections = 0;
  std::string error;
  const auto check = [&](auto &) {++checks; canceled = true; return true;};
  const auto correct = [&](auto &) {++corrections; return true;};
  EXPECT_FALSE(checkpoint.validate([](auto &) {return true;}, check, correct, [&] {return canceled;}, error));
  EXPECT_EQ(checks, 0);
  canceled = false;
  EXPECT_FALSE(checkpoint.validate([](auto &) {return true;}, check, correct, [&] {return canceled;}, error));
  EXPECT_EQ(checks, 1);
  EXPECT_EQ(corrections, 0);
}

TEST(ReleaseCheckpoint, ExitTimeoutThenContinueCorrectsBeforeRechecking)
{
  PhaseRetryController controller;
  controller.begin("task", "place", "attached");
  ReleaseCheckpoint checkpoint;
  int checks = 0, corrections = 0, releases = 0, pauses = 0;
  bool abort = false;
  std::string error, request_error;
  EXPECT_TRUE(controller.run("release", false, 3, 0.05, 0.0,
    [&](auto deadline, std::string & failure) {
      const auto interrupted = [&] {
          return PhaseRetryController::Clock::now() >= deadline;
        };
      const bool valid = checkpoint.validate([](auto &) {return true;}, [&](auto & check_error) {
          ++checks;
          if (checks == 1) {
            std::this_thread::sleep_until(deadline);
            check_error = "measured exit deadline exhausted";
            return false;
          }
          EXPECT_EQ(corrections, 1);
          return true;
        }, [&](auto &) {++corrections; return true;}, interrupted, failure);
      if (valid) {++releases;}
      return valid;
    }, [&] {return abort;}, [&](const auto & status) {
      if (status.status == "paused") {
        ++pauses;
        EXPECT_EQ(releases, 0);
        EXPECT_EQ(corrections, 0);
        EXPECT_EQ(status.object_disposition, "attached");
        EXPECT_EQ(status.failure, "measured exit deadline exhausted");
        if (pauses > 1) {abort = true; return;}
        EXPECT_TRUE(controller.requestContinue("task", status.pause_id, request_error));
      }
    }, error));
  EXPECT_EQ(pauses, 1);
  EXPECT_EQ(checks, 2);
  EXPECT_EQ(corrections, 1);
  EXPECT_EQ(releases, 1);
}

}  // namespace
}  // namespace agibot_x2_manipulation
