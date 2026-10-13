#include "pick_place/endpoint_precheck_diagnostics.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <limits>

namespace agibot_x2_manipulation
{
namespace
{

moveit::core::RobotModelPtr testModel()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="precheck"><link name="base"/><link name="left"/><link name="right"/>
      <link name="tip"/>
      <joint name="left_joint" type="prismatic"><parent link="base"/><child link="left"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="right_joint" type="prismatic"><parent link="left"/><child link="right"/>
        <limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <joint name="continuous_joint" type="continuous"><parent link="right"/><child link="tip"/>
        <limit effort="10" velocity="1"/></joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(
    <robot name="precheck"><group name="dual_arm"><chain base_link="base" tip_link="tip"/>
    </group></robot>)");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

TEST(EndpointPrecheckDiagnostics, ReportsBothBoundViolationsWithValuesAndLimits)
{
  const auto model = testModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePosition("left_joint", -1.25);
  state.setVariablePosition("right_joint", 2.5);
  const auto * group = model->getJointModelGroup("dual_arm");
  ASSERT_FALSE(state.satisfiesBounds(group));
  EndpointPrecheckDiagnostics diagnostics;
  diagnostics.rejectBounds(state, group);
  EXPECT_EQ(diagnostics.bounds_rejected, 1U);
  EXPECT_EQ(diagnostics.bounds_examples.size(), 2U);
  EXPECT_NE(diagnostics.summary().find("left_joint=-1.25 (limits=[-1, 1])"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("right_joint=2.5 (limits=[-2, 2])"), std::string::npos);
  state.setVariablePosition("left_joint", -1.5);
  diagnostics.rejectBounds(state, group);
  EXPECT_EQ(diagnostics.bounds_examples.size(), 2U);
  EXPECT_NE(diagnostics.summary().find("left_joint=-1.25"), std::string::npos);
}

TEST(EndpointPrecheckDiagnostics, DoesNotBlameValidLimitsOrContinuousJoints)
{
  const auto model = testModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePosition("left_joint", -1.0);
  state.setVariablePosition("right_joint", 2.0);
  state.setVariablePosition("continuous_joint", 20.0);
  EXPECT_TRUE(positionBoundsViolations(state, model->getJointModelGroup("dual_arm")).empty());
  state.setVariablePosition("right_joint", std::numeric_limits<double>::quiet_NaN());
  const auto violations = positionBoundsViolations(state, model->getJointModelGroup("dual_arm"));
  ASSERT_EQ(violations.size(), 1U);
  EXPECT_EQ(violations.begin()->first, "right_joint");
}

TEST(EndpointPrecheckDiagnostics, DistinguishesFailedIKArmsFromInterruptedSolves)
{
  EndpointPrecheckDiagnostics diagnostics;
  diagnostics.candidates = 81;
  diagnostics.evaluated = 4;
  diagnostics.rejectIK(DualArmIKFailure::LEFT_ARM);
  diagnostics.rejectIK(DualArmIKFailure::RIGHT_ARM);
  diagnostics.rejectIK(DualArmIKFailure::DEADLINE);
  diagnostics.rejectIK(DualArmIKFailure::CANCELED);
  diagnostics.stop_reason = "phase_deadline";
  EXPECT_EQ(diagnostics.ik_rejected, 2U);
  EXPECT_EQ(diagnostics.ik_interrupted, 2U);
  EXPECT_NE(diagnostics.summary().find("IK=2 (left=1, right=1)"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("IK_interrupted=2"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("evaluated=4/81"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("stop_reason=phase_deadline"), std::string::npos);
}

TEST(EndpointPrecheckDiagnostics, PreservesDistinctCollisionPairsAndDeduplicatesRepeats)
{
  EndpointPrecheckDiagnostics diagnostics;
  diagnostics.rejectCollision("left_wrist <-> table");
  diagnostics.rejectCollision("left_wrist <-> table");
  diagnostics.rejectCollision("right_wrist <-> box");
  EXPECT_EQ(diagnostics.collision_rejected, 3U);
  EXPECT_EQ(diagnostics.colliding_pairs.size(), 2U);
  const auto message = diagnostics.summary();
  EXPECT_NE(message.find("left_wrist <-> table"), std::string::npos);
  EXPECT_NE(message.find("right_wrist <-> box"), std::string::npos);
  diagnostics.rejectCollision("");
  EXPECT_NE(diagnostics.summary().find("contact pairs unavailable"), std::string::npos);
}

TEST(EndpointPrecheckDiagnostics, DeadlineBeforeFirstCandidateDoesNotClaimIKFailure)
{
  EndpointPrecheckDiagnostics diagnostics;
  diagnostics.candidates = 81;
  diagnostics.stop_reason = "search_deadline";
  EXPECT_NE(diagnostics.summary().find("evaluated=0/81"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("IK=0 (left=0, right=0)"), std::string::npos);
  EXPECT_NE(diagnostics.summary().find("stop_reason=search_deadline"), std::string::npos);
  EXPECT_TRUE(diagnostics.bounds_examples.empty());
  EXPECT_TRUE(diagnostics.colliding_pairs.empty());
}

}  // namespace
}  // namespace agibot_x2_manipulation
