#include "agibot_x2_manipulation/box_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace agibot_x2_manipulation
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d boxAtYaw(double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

TEST(BoxGeometry, TopTagIsShiftedToBoxCenter)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 0.8);
  const auto box = boxPoseFromTopTag(tag, {0.4, 0.2, 0.3});
  EXPECT_NEAR(box.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(box.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(box.translation().z(), 0.65, 1e-12);
}

TEST(BoxGeometry, AlignedBoxUsesLocalYFaces)
{
  const auto grasp = computeGraspGeometry(boxAtYaw(0.0), {0.4, 0.2, 0.3}, 0.08);
  EXPECT_EQ(grasp.selected_axis, 'y');
  EXPECT_NEAR(grasp.left_contact.translation().y(), 0.1, 1e-12);
  EXPECT_NEAR(grasp.right_contact.translation().y(), -0.1, 1e-12);
  EXPECT_NEAR(grasp.left_pregrasp.translation().y(), 0.18, 1e-12);
  const Eigen::Vector3d left_minus_y = grasp.left_contact.linear() * -Eigen::Vector3d::UnitY();
  const Eigen::Vector3d right_plus_y = grasp.right_contact.linear() * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d left_plus_x = grasp.left_contact.linear() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d right_plus_x = grasp.right_contact.linear() * Eigen::Vector3d::UnitX();
  EXPECT_GT(left_minus_y.dot(-Eigen::Vector3d::UnitY()), 0.999);
  EXPECT_GT(right_plus_y.dot(Eigen::Vector3d::UnitY()), 0.999);
  EXPECT_GT(left_plus_x.dot(Eigen::Vector3d::UnitZ()), 0.999);
  EXPECT_GT(right_plus_x.dot(Eigen::Vector3d::UnitZ()), 0.999);
}

TEST(BoxGeometry, QuarterTurnUsesLocalXFacesAndAssignsRobotLeft)
{
  const auto grasp = computeGraspGeometry(boxAtYaw(kPi / 2.0), {0.4, 0.2, 0.3}, 0.08);
  EXPECT_EQ(grasp.selected_axis, 'x');
  EXPECT_GT(grasp.left_contact.translation().y(), 0.0);
  EXPECT_LT(grasp.right_contact.translation().y(), 0.0);
  EXPECT_GT(grasp.left_outward_normal.dot(Eigen::Vector3d::UnitY()), 0.99);
}

TEST(BoxGeometry, ExactDiagonalTieUsesLocalY)
{
  const auto grasp = computeGraspGeometry(boxAtYaw(kPi / 4.0), {0.4, 0.2, 0.3}, 0.08);
  EXPECT_EQ(grasp.selected_axis, 'y');
  EXPECT_GT(grasp.left_contact.translation().y(), 0.0);
  EXPECT_LT(grasp.right_contact.translation().y(), 0.0);
}

TEST(BoxGeometry, HalfTurnStillAssignsRobotLeft)
{
  const auto grasp = computeGraspGeometry(boxAtYaw(kPi), {0.4, 0.2, 0.3}, 0.08);
  EXPECT_EQ(grasp.selected_axis, 'y');
  EXPECT_GT(grasp.left_contact.translation().y(), 0.0);
  EXPECT_LT(grasp.right_contact.translation().y(), 0.0);
}

TEST(BoxGeometry, InvalidContactHeightIsRejected)
{
  EXPECT_THROW(
    computeGraspGeometry(Eigen::Isometry3d::Identity(), {0.4, 0.2, 0.3}, 0.08, 0.15),
    std::invalid_argument);
}

TEST(BoxGeometry, PelvisRelativeDummyPoseProducesExpectedGrasps)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(0.35, 0.0, 0.45);
  const BoxDimensions dimensions{0.15, 0.35, 0.32};
  const auto box = boxPoseFromTopTag(tag, dimensions);
  const auto grasp = computeGraspGeometry(box, dimensions, 0.08);

  EXPECT_NEAR(box.translation().x(), 0.35, 1e-12);
  EXPECT_NEAR(box.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(box.translation().z(), 0.29, 1e-12);
  EXPECT_EQ(grasp.selected_axis, 'y');
  EXPECT_NEAR(grasp.left_contact.translation().y(), 0.175, 1e-12);
  EXPECT_NEAR(grasp.right_contact.translation().y(), -0.175, 1e-12);
  EXPECT_NEAR(grasp.left_pregrasp.translation().y(), 0.255, 1e-12);
  EXPECT_NEAR(grasp.right_pregrasp.translation().y(), -0.255, 1e-12);
  EXPECT_NEAR(grasp.left_pregrasp.translation().z(), 0.29, 1e-12);
  EXPECT_NEAR(grasp.right_pregrasp.translation().z(), 0.29, 1e-12);
}

}  // namespace
}  // namespace agibot_x2_manipulation
