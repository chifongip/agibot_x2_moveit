#include "pick_place/post_place_progress.hpp"
#include "agibot_x2_manipulation/phase_retry_controller.hpp"

#include <gtest/gtest.h>

namespace agibot_x2_manipulation
{
namespace
{

PostPlaceSegment segment(const std::string & name, bool retreat = false)
{
  PostPlaceSegment result;
  result.name = name;
  result.retreat = retreat;
  return result;
}

TEST(PostPlaceProgress, ContinueAfterFailedReplanExecutesPrepareAndReady)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  ASSERT_TRUE(progress.replan([](auto & plan) {
    plan.segments = {segment("retreat", true), segment("to_prepare"), segment("from_prepare_to_ready")};
    return true;
  }, error));
  progress.advance();
  EXPECT_FALSE(progress.include_retreat());
  EXPECT_TRUE(progress.include_prepare());

  // The pending Prepare segment becomes invalid. The failed planner clears
  // its output, as planPostPlaceSequence does on a colliding Ready target.
  ASSERT_FALSE(progress.replan([&](auto & plan) {
    plan.segments.clear();
    error = "return named target invalid: collision";
    return false;
  }, error));
  EXPECT_EQ(progress.current(), nullptr);
  EXPECT_FALSE(progress.complete());

  PhaseRetryController retry;
  retry.begin("place-task", "place", "released");
  retry.checkpoint("retreat", "released");
  bool resumed = false;
  int planning_calls = 0;
  std::string request_error;
  ASSERT_TRUE(retry.run("to_prepare", false, 1, 1.0, 0.0,
    [&](auto, std::string & failure) {
      return progress.replan([&](auto & plan) {
        ++planning_calls;
        if (!resumed) {failure = "Ready still collides"; return false;}
        plan.segments = {segment("to_prepare"), segment("from_prepare_to_ready")};
        return true;
      }, failure);
    }, []() {return false;}, [&](const auto & status) {
      if (status.status == "paused") {
        EXPECT_EQ(status.last_completed_phase, "retreat");
        EXPECT_EQ(status.object_disposition, "released");
        EXPECT_FALSE(progress.complete());
        resumed = true;
        EXPECT_TRUE(retry.requestContinue("place-task", status.pause_id, request_error));
      }
    }, error)) << error;
  EXPECT_EQ(planning_calls, 2);
  ASSERT_NE(progress.current(), nullptr);
  EXPECT_EQ(progress.current()->name, "to_prepare");
  progress.advance();
  EXPECT_FALSE(progress.complete());
  EXPECT_FALSE(progress.include_prepare());
  ASSERT_NE(progress.current(), nullptr);
  EXPECT_EQ(progress.current()->name, "from_prepare_to_ready");
  progress.advance();
  EXPECT_TRUE(progress.complete());
}

TEST(PostPlaceProgress, FailedReadyExecutionResumesWithoutRepeatingRetreatOrPrepare)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  ASSERT_TRUE(progress.replan([](auto & plan) {
    plan.segments = {segment("retreat", true), segment("to_prepare"), segment("from_prepare_to_ready")};
    return true;
  }, error));
  progress.advance();
  progress.advance();
  progress.invalidate();
  EXPECT_FALSE(progress.complete());
  EXPECT_FALSE(progress.include_retreat());
  EXPECT_FALSE(progress.include_prepare());
  ASSERT_FALSE(progress.replan([](auto & plan) {
    plan.segments = {segment("partial_failed_plan")};
    return false;
  }, error));
  EXPECT_EQ(progress.current(), nullptr);
  EXPECT_THROW(progress.advance(), std::logic_error);
  ASSERT_TRUE(progress.replan([](auto & plan) {
    plan.segments = {segment("ready_direct")};
    return true;
  }, error));
  EXPECT_EQ(progress.current()->name, "ready_direct");
  progress.advance();
  EXPECT_TRUE(progress.complete());
}

TEST(PostPlaceProgress, EmptyCandidateCannotCompleteTask)
{
  PostPlaceProgress progress("prepare");
  std::string error;
  EXPECT_FALSE(progress.complete());
  EXPECT_FALSE(progress.replan([](auto &) {return true;}, error));
  EXPECT_EQ(error, "empty return sequence");
  EXPECT_FALSE(progress.complete());
  EXPECT_EQ(progress.current(), nullptr);
}

}  // namespace
}  // namespace agibot_x2_manipulation
