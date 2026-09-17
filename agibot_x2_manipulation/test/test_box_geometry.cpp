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

TEST(BoxGeometry, TopTagAppliesTagFrameOffset)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 0.8);
  tag.linear() = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d offset(0.10, -0.20, 0.05);

  const auto box = boxPoseFromTopTag(tag, {0.4, 0.2, 0.3}, 0.0, offset);

  EXPECT_LT(
    (box.translation() - tag * Eigen::Vector3d(0.10, -0.20, -0.10)).norm(),
    1e-12);
}

TEST(BoxGeometry, VerticalTableTagDerivesAnUprightPlacePose)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto box = boxPoseFromVerticalTableTag(tag, {0.4, 0.2, 0.3}, 0.55);

  EXPECT_NEAR(box.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(box.translation().y(), 1.60, 1e-12);
  EXPECT_NEAR(box.translation().z(), 3.0, 1e-12);
  EXPECT_LT((box.linear() * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ()).norm(), 1e-12);
  EXPECT_LT((box.linear() * Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitX()).norm(), 1e-12);
  EXPECT_LT((box.linear() * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitY()).norm(), 1e-12);
}

TEST(BoxGeometry, VerticalTableTagAppliesTabletopOffsetsAndBoxYaw)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.linear() = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto box = boxPoseFromVerticalTableTag(
    tag, {0.4, 0.2, 0.3}, 0.55, 0.2, -0.1, kPi / 2.0);

  EXPECT_LT((box.translation() - tag * Eigen::Vector3d(0.2, -0.4, -0.1)).norm(), 1e-12);
  EXPECT_LT(
    (box.linear() - tag.linear() *
    Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix() *
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm(),
    1e-12);
}

TEST(BoxGeometry, TablePlacementIsIndependentOfPickupTagCalibration)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto box = boxPoseFromVerticalTableTag(
    tag, {0.4, 0.2, 0.3}, 0.55, 0.0, 0.0, kPi / 2.0);

  const Eigen::Matrix3d table_tag_to_box =
    Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix() *
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_LT(
    (box.translation() - tag * Eigen::Vector3d(0.0, -0.4, 0.0)).norm(),
    1e-12);
  EXPECT_LT((box.linear() - tag.linear() * table_tag_to_box).norm(), 1e-12);
}

TEST(BoxGeometry, VerticalTableTagBuildsTheCalibratedTableCollisionPose)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto table = tablePoseFromVerticalTag(
    tag, {0.5, 0.3, 0.6}, Eigen::Vector3d(0.0, -0.55, 0.15));

  EXPECT_LT(
    (table.translation() - tag * Eigen::Vector3d(0.0, -0.85, 0.15)).norm(), 1e-12);
  EXPECT_LT((table.linear() * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX()).norm(),
    1e-12);
  EXPECT_LT((table.linear() * Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitZ()).norm(),
    1e-12);
  EXPECT_LT((table.linear() * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitY()).norm(),
    1e-12);
}

TEST(BoxGeometry, BottomTagUsesAnExplicitTagToCenterTransform)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  tag.translation() = Eigen::Vector3d(1.0, 2.0, 0.5);
  Eigen::Isometry3d tag_to_box_center = Eigen::Isometry3d::Identity();
  tag_to_box_center.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);

  const auto box = boxPoseFromTag(tag, tag_to_box_center);

  EXPECT_LT(
    (box.translation() - Eigen::Vector3d(1.0, 2.0, 0.65)).norm(),
    1e-12);
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

TEST(BoxGeometry, ContactHeightOutsideTheBoxIsAllowed)
{
  const auto grasp = computeGraspGeometry(
    Eigen::Isometry3d::Identity(), {0.4, 0.2, 0.3}, 0.08, 0.20);

  EXPECT_NEAR(grasp.left_contact.translation().z(), 0.20, 1e-12);
  EXPECT_NEAR(grasp.right_contact.translation().z(), 0.20, 1e-12);
}

TEST(BoxGeometry, CandidateGenerationAllowsContactHeightOutsideTheBox)
{
  GraspCandidateOptions options;
  options.position_tolerance = 0.0;
  options.orientation_tolerance = 0.0;
  options.pregrasp_distance_tolerance = 0.0;
  options.alternate_face_alignment_tolerance = 0.0;
  options.maximum_candidates = 1;

  const auto candidates = generateGraspCandidates(
    Eigen::Isometry3d::Identity(), {0.4, 0.2, 0.3}, 0.08, -0.20, options);

  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().contact_height_offset, -0.20, 1e-12);
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

TEST(BoxGeometry, CandidateSearchStartsWithUnmodifiedCoordinatedGrasp)
{
  Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
  box.translation() = Eigen::Vector3d(0.3, 0.0, 0.2);
  GraspCandidateOptions options;
  options.maximum_candidates = 64;
  const auto candidates = generateGraspCandidates(box, {0.15, 0.35, 0.32}, 0.08, 0.0, options);

  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().correction_cost, 0.0, 1e-12);
  EXPECT_NEAR(candidates.front().tilt_correction, 0.0, 1e-12);
  EXPECT_NEAR(candidates.front().contact_height_offset, 0.0, 1e-12);
  EXPECT_NEAR(candidates.front().tangent_offset, 0.0, 1e-12);
  EXPECT_NEAR(candidates.front().wrist_rotation, 0.0, 1e-12);
  EXPECT_NEAR(candidates.front().pregrasp_distance, 0.08, 1e-12);
}

TEST(BoxGeometry, TiltCorrectionKeepsMeasuredTopCenterFixed)
{
  Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
  box.translation() = Eigen::Vector3d(0.32, 0.01, 0.12);
  box.linear() = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const BoxDimensions dimensions{0.15, 0.35, 0.32};
  GraspCandidateOptions options;
  options.position_tolerance = 0.0;
  options.orientation_tolerance = 0.0872664626;
  options.pregrasp_distance_tolerance = 0.0;
  options.maximum_candidates = 500;
  const auto candidates = generateGraspCandidates(box, dimensions, 0.08, 0.0, options);
  const Eigen::Vector3d measured_top =
    box.translation() + box.linear() * Eigen::Vector3d(0.0, 0.0, dimensions.height / 2.0);

  ASSERT_GE(candidates.size(), 3U);
  bool found_maximum_correction = false;
  for (const auto & candidate : candidates) {
    const Eigen::Vector3d candidate_top = candidate.planning_box_pose.translation() +
      candidate.planning_box_pose.linear() *
      Eigen::Vector3d(0.0, 0.0, dimensions.height / 2.0);
    EXPECT_LT((candidate_top - measured_top).norm(), 1e-10);
    if (std::abs(candidate.tilt_correction - options.orientation_tolerance) < 1e-10) {
      found_maximum_correction = true;
    }
  }
  EXPECT_TRUE(found_maximum_correction);
}

TEST(BoxGeometry, CandidateOffsetsRemainCoordinatedAndBounded)
{
  GraspCandidateOptions options;
  options.maximum_candidates = 64;
  const auto candidates = generateGraspCandidates(
    Eigen::Isometry3d::Identity(), {0.15, 0.35, 0.32}, 0.08, 0.0, options);

  ASSERT_EQ(candidates.size(), options.maximum_candidates);
  for (const auto & candidate : candidates) {
    EXPECT_LE(std::abs(candidate.contact_height_offset), options.position_tolerance + 1e-12);
    EXPECT_LE(std::abs(candidate.tangent_offset), options.position_tolerance + 1e-12);
    EXPECT_LE(std::abs(candidate.wrist_rotation), options.orientation_tolerance + 1e-12);
    EXPECT_LE(
      std::abs(candidate.pregrasp_distance - 0.08),
      options.pregrasp_distance_tolerance + 1e-12);
    const Eigen::Vector3d left_inward =
      candidate.grasp.left_contact.linear() * -Eigen::Vector3d::UnitY();
    const Eigen::Vector3d right_inward =
      candidate.grasp.right_contact.linear() * Eigen::Vector3d::UnitY();
    EXPECT_LT(left_inward.dot(candidate.grasp.left_outward_normal), -0.999);
    EXPECT_LT(right_inward.dot(candidate.grasp.right_outward_normal), -0.999);
    EXPECT_LT(
      (candidate.box_to_left_contact.matrix() - candidate.grasp.left_contact.matrix()).norm(),
      1e-10);
    EXPECT_LT(
      (candidate.box_to_right_contact.matrix() - candidate.grasp.right_contact.matrix()).norm(),
      1e-10);
  }
}

}  // namespace
}  // namespace agibot_x2_manipulation
