#include "pick_place/carry_pose_frame.hpp"
#include "pick_place/saved_plan.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace agibot_x2_manipulation
{
namespace
{

moveit::core::RobotModelPtr postureModel()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="carry">
      <link name="world"/><link name="base_link"/><link name="yaw"/>
      <link name="pitch"/><link name="torso_link"/>
      <joint name="height" type="prismatic">
        <parent link="world"/><child link="base_link"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="1" effort="10" velocity="1"/>
      </joint>
      <joint name="waist_yaw" type="revolute">
        <parent link="base_link"/><child link="yaw"/><origin xyz="0 0 0.1"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
      <joint name="waist_pitch" type="revolute">
        <parent link="yaw"/><child link="pitch"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
      <joint name="waist_roll" type="revolute">
        <parent link="pitch"/><child link="torso_link"/><origin xyz="0 0 0.2"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, "<robot name='carry'/>");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

TEST(CarryPoseFrame, PositionAndOrientationFollowTorsoAtBothHeights)
{
  moveit::core::RobotState state(postureModel());
  state.setToDefaultValues();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.3, -0.02, 0.2);
  target.linear() = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (double height : {0.4, 0.6}) {
    for (const Eigen::Vector3d & tilt : {Eigen::Vector3d::Zero().eval(),
      Eigen::Vector3d(0.3, -0.2, 0.1)})
    {
      state.setVariablePosition("height", height);
      state.setVariablePosition("waist_yaw", tilt.x());
      state.setVariablePosition("waist_pitch", tilt.y());
      state.setVariablePosition("waist_roll", tilt.z());
      state.update();
      const auto rotation = (Eigen::AngleAxisd(tilt.x(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(tilt.y(), Eigen::Vector3d::UnitY())).toRotationMatrix();
      Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
      expected.linear() = rotation *
        Eigen::AngleAxisd(tilt.z(), Eigen::Vector3d::UnitX()).toRotationMatrix();
      expected.translation() = Eigen::Vector3d(0, 0, height + 0.1) +
        rotation * Eigen::Vector3d(0, 0, 0.2);
      const auto achieved = carryPoseInPlanningFrame(state, "world", target);
      EXPECT_TRUE(achieved.matrix().isApprox((expected * target).matrix(), 1e-12));
      EXPECT_TRUE((carryFrameTransform(state, "world").inverse() * achieved).matrix().isApprox(
          target.matrix(), 1e-12));
      // Height cancels in pelvis coordinates; waist tilt does not.
      expected.translation().z() -= height;
      EXPECT_TRUE(carryPoseInPlanningFrame(state, "base_link", target).matrix().isApprox(
          (expected * target).matrix(), 1e-12));
    }
  }
}

TEST(CarryPoseFrame, RememberedEndpointsFollowNewPosture)
{
  moveit::core::RobotState state(postureModel());
  state.setToDefaultValues();
  state.update();
  const auto original_frame = carryFrameTransform(state, "world");
  Eigen::Isometry3d selected = Eigen::Isometry3d::Identity();
  selected.translation() = Eigen::Vector3d(0.32, 0.02, 0.18);
  const auto stored = original_frame.inverse() * (original_frame * selected);
  state.setVariablePosition("height", 0.6);
  state.setVariablePosition("waist_pitch", -0.3);
  state.update();
  EXPECT_TRUE(carryPoseInPlanningFrame(state, "world", stored).matrix().isApprox(
      (carryFrameTransform(state, "world") * selected).matrix(), 1e-12));
  EXPECT_FALSE(carryPoseInPlanningFrame(state, "world", stored).matrix().isApprox(
      (original_frame * selected).matrix(), 1e-3));
}

TEST(CarryPoseFrame, FreshFeedbackWithPendingForwardKinematicsIsSupported)
{
  moveit::core::RobotState state(postureModel());
  state.setToDefaultValues();
  state.update();
  state.setVariablePosition("waist_pitch", 0.2);
  ASSERT_TRUE(state.dirtyLinkTransforms());
  const auto resolved = carryFrameTransform(state, "base_link");
  EXPECT_TRUE(state.dirtyLinkTransforms());
  state.update();
  EXPECT_TRUE(resolved.matrix().isApprox(
      carryFrameTransform(state, "base_link").matrix(), 1e-12));
}

TEST(CarryPoseFrame, SavedPreviewRejectsChangedTorsoAndAcceptsFreshFrame)
{
  moveit::core::RobotState state(postureModel());
  state.setToDefaultValues();
  state.update();
  SavedPlan plan;
  plan.config.planning_frame = "world";
  plan.config.closed_chain_contact_position_error = 0.01;
  plan.config.closed_chain_contact_orientation_error = 0.02;
  plan.carry_frame = carryFrameTransform(state, "world");
  std::string error;
  EXPECT_TRUE(validateSavedCarryFrame(plan, state, error));
  state.setVariablePosition("height", 0.2);
  state.update();
  EXPECT_FALSE(validateSavedCarryFrame(plan, state, error));
  EXPECT_NE(error.find("fresh plan-only preview"), std::string::npos);
  plan.carry_frame = carryFrameTransform(state, "world");
  EXPECT_TRUE(validateSavedCarryFrame(plan, state, error));
  state.setVariablePosition("waist_roll", 0.2);
  state.update();
  EXPECT_FALSE(validateSavedCarryFrame(plan, state, error));
}

TEST(CarryPoseFrame, MissingReferenceFramesFailClearly)
{
  moveit::core::RobotState state(postureModel());
  state.setToDefaultValues();
  state.update();
  EXPECT_THROW(carryFrameTransform(state, "missing"), std::runtime_error);
  const auto urdf = urdf::parseURDF("<robot name='missing'><link name='base_link'/></robot>");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, "<robot name='missing'/>");
  moveit::core::RobotState missing(std::make_shared<moveit::core::RobotModel>(urdf, srdf));
  missing.setToDefaultValues();
  missing.update();
  EXPECT_THROW(carryFrameTransform(missing, "base_link"), std::runtime_error);
}

}  // namespace
}  // namespace agibot_x2_manipulation
