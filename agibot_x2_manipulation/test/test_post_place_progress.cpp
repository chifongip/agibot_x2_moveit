#include "pick_place/post_place_progress.hpp"
#include "agibot_x2_manipulation/phase_retry_controller.hpp"

#include <gtest/gtest.h>

namespace agibot_x2_manipulation
{
namespace
{

bool planStage(PostPlaceProgress & progress, std::string & error, int segments = 1)
{
  return progress.replan([segments](auto & plan) {
    plan.segments.resize(segments);
    return true;
  }, error);
}

void completeStage(PostPlaceProgress & progress, std::string & error)
{
  ASSERT_TRUE(planStage(progress, error));
  progress.advance_segment();
  progress.advance_stage();
}

TEST(PostPlaceProgress, ReadyCollisionDoesNotBlockCompletedRetreatAndPrepare)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  completeStage(progress, error);
  EXPECT_EQ(progress.stage(), PostPlaceStage::PREPARE);
  completeStage(progress, error);
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  EXPECT_FALSE(progress.replan([&](auto & plan) {
    plan.segments.resize(1);  // Partial output from a failed planner is not executable.
    error = "return named target invalid: collision";
    return false;
  }, error));
  EXPECT_EQ(progress.current(), nullptr);
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  EXPECT_FALSE(progress.complete());
  EXPECT_THROW(progress.advance_stage(), std::logic_error);
}

TEST(PostPlaceProgress, ContinueRetriesPrepareWithoutRepeatingRetreat)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  completeStage(progress, error);
  PhaseRetryController retry;
  retry.begin("place-task", "place", "released");
  retry.checkpoint("retreat", "released");
  bool resumed = false;
  int planning_calls = 0;
  std::string request_error;
  ASSERT_TRUE(retry.run("to_prepare", false, 1, 1.0, 0.0,
    [&](auto, std::string & failure) {
      EXPECT_EQ(progress.stage(), PostPlaceStage::PREPARE);
      ++planning_calls;
      if (!resumed) {
        return progress.replan([&](auto &) {
          failure = "Prepare collides";
          return false;
        }, failure);
      }
      if (!planStage(progress, failure)) {return false;}
      progress.advance_segment();
      return progress.stage_complete();
    }, []() {return false;}, [&](const auto & status) {
      if (status.status == "paused") {
        EXPECT_EQ(status.last_completed_phase, "retreat");
        EXPECT_EQ(status.object_disposition, "released");
        EXPECT_FALSE(retry.requestContinue("wrong-task", status.pause_id, request_error));
        EXPECT_FALSE(retry.requestContinue("place-task", status.pause_id + 1, request_error));
        resumed = true;
        EXPECT_TRUE(retry.requestContinue("place-task", status.pause_id, request_error));
        EXPECT_FALSE(retry.requestContinue("place-task", status.pause_id, request_error));
      }
    }, error)) << error;
  EXPECT_EQ(planning_calls, 2);
  progress.advance_stage();
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  completeStage(progress, error);
  EXPECT_TRUE(progress.complete());
}

TEST(PostPlaceProgress, PartialExecutionAndFailedReplansKeepActiveStage)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  completeStage(progress, error);
  completeStage(progress, error);
  ASSERT_TRUE(planStage(progress, error, 2));
  progress.advance_segment();
  EXPECT_FALSE(progress.stage_complete());
  progress.invalidate();
  EXPECT_EQ(progress.current(), nullptr);
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  EXPECT_FALSE(progress.replan([](auto &) {return false;}, error));
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  ASSERT_TRUE(planStage(progress, error));
  progress.advance_segment();
  progress.advance_stage();
  EXPECT_TRUE(progress.complete());
}

TEST(PostPlaceProgress, EmptyPrepareIsSkippedAndNoMotionStageCanComplete)
{
  PostPlaceProgress progress("");
  std::string error;
  ASSERT_TRUE(progress.replan([](auto & plan) {
    PostPlaceSegment segment;
    segment.no_motion = true;
    plan.segments.push_back(segment);
    return true;
  }, error));
  progress.advance_segment();
  progress.advance_stage();
  EXPECT_EQ(progress.stage(), PostPlaceStage::READY);
  completeStage(progress, error);
  EXPECT_TRUE(progress.complete());
}

TEST(PostPlaceProgress, EmptyCandidateCannotCompleteStage)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  EXPECT_FALSE(progress.replan([](auto &) {return true;}, error));
  EXPECT_EQ(error, "empty post-place stage");
  EXPECT_EQ(progress.current(), nullptr);
  EXPECT_FALSE(progress.complete());
  EXPECT_THROW(progress.advance_segment(), std::logic_error);
  EXPECT_THROW(progress.advance_stage(), std::logic_error);
}

}  // namespace
}  // namespace agibot_x2_manipulation
