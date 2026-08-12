#include "agibot_x2_manipulation/execution_feedback.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>

namespace agibot_x2_manipulation
{
namespace
{

TEST(ExecutionFeedback, RejectsOffsetAndMovingMeasurements)
{
  const std::map<std::string, double> target{{"left_elbow_joint", 0.5}};

  const auto offset = checkExecutionFeedback(
    target, {{"left_elbow_joint", {0.54, 0.0}}}, 0.02, 0.01);
  EXPECT_FALSE(offset.settled);
  EXPECT_EQ(offset.worst_position_joint, "left_elbow_joint");

  const auto moving = checkExecutionFeedback(
    target, {{"left_elbow_joint", {0.5, 0.05}}}, 0.02, 0.01);
  EXPECT_FALSE(moving.settled);
  EXPECT_EQ(moving.worst_velocity_joint, "left_elbow_joint");

  const auto settled = checkExecutionFeedback(
    target, {{"left_elbow_joint", {0.51, 0.005}}}, 0.02, 0.01);
  EXPECT_TRUE(settled.settled);
}

TEST(ExecutionFeedback, RequiresASequenceNewerThanTheCompletionBaseline)
{
  constexpr uint32_t sequence = 42U;
  EXPECT_FALSE(isNewerMeasurementSequence(sequence, sequence));
  EXPECT_FALSE(isNewerMeasurementSequence(sequence - 1U, sequence));
  EXPECT_TRUE(isNewerMeasurementSequence(sequence + 1U, sequence));
  EXPECT_TRUE(isNewerMeasurementSequence(0U, UINT32_MAX));
}

}  // namespace
}  // namespace agibot_x2_manipulation
