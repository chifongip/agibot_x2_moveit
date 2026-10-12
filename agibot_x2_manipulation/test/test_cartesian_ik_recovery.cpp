#include "pick_place/cartesian_motion.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <cmath>
#include <sstream>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{
struct RecoveryFixture
{
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  PickPlaceConfig config;
  std::unique_ptr<moveit::core::RobotState> start;
  HandPosePair target;
  moveit_msgs::msg::RobotTrajectory output;
  std::unique_ptr<moveit::core::RobotState> end;
  std::string error;

  RecoveryFixture()
  {
    // Each arm has two redundant X slides and one Z slide. Alternative joint
    // branches can follow exactly the same TCP path without a real IK plugin.
    std::ostringstream xml;
    xml << "<robot name='recovery'><link name='base'/>";
    for (const auto & arm : {std::string("left"), std::string("right")}) {
      xml << "<link name='" << arm << "_base'/><joint name='" << arm << "_fixed' type='fixed'>"
          << "<parent link='base'/><child link='" << arm << "_base'/><origin xyz='0 "
          << (arm == "left" ? 0 : 1) << " 0'/></joint>";
      for (int i = 0; i < 3; ++i) {
        xml << "<link name='" << arm << i << "'>";
        if (i == 2) {
          xml << "<collision><geometry><box size='0.01 0.01 0.01'/></geometry></collision>";
        }
        xml << "</link><joint name='" << arm << "_q" << i << "' type='prismatic'>"
            << "<parent link='" << arm << (i == 0 ? "_base" : std::to_string(i - 1))
            << "'/><child link='" << arm << i << "'/><axis xyz='"
            << (i == 2 ? "0 0 1" : "1 0 0")
            << "'/><limit lower='-2' upper='2' effort='10' velocity='1'/></joint>";
      }
    }
    xml << "</robot>";
    const auto urdf = urdf::parseURDF(xml.str());
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*urdf, R"(<robot name="recovery">
      <group name="left"><chain base_link="left_base" tip_link="left2"/></group>
      <group name="right"><chain base_link="right_base" tip_link="right2"/></group>
      <group name="dual"><group name="left"/><group name="right"/></group></robot>)");
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    start = std::make_unique<moveit::core::RobotState>(model);
    start->setToDefaultValues();
    start->update();
    end = std::make_unique<moveit::core::RobotState>(*start);
    config.planning_group = "dual";
    config.left_group_name = "left"; config.right_group_name = "right";
    config.left_tcp = "left2"; config.right_tcp = "right2";
    config.cartesian_step = 0.01;
    config.closed_chain_orientation_step = 0.0261799388;
    config.closed_chain_ik_timeout = 0.1;
    config.max_joint_step = 0.35;
    config.return_validation_joint_step = 0.02;
    config.velocity_scaling = config.acceleration_scaling = 0.2;
    target = {start->getGlobalLinkTransform("left2"), start->getGlobalLinkTransform("right2")};
    target.left.translation().x() = target.right.translation().x() = 0.04;
    output.joint_trajectory.joint_names = {"sentinel"};
    end->setVariablePosition("left_q0", 0.123);
  }

  static bool solve(moveit::core::RobotState & state, const moveit::core::JointModelGroup * arm,
    const Eigen::Isometry3d & pose, double branch = 0.0)
  {
    const auto & names = arm->getVariableNames();
    state.setVariablePosition(names[0], branch);
    state.setVariablePosition(names[1], pose.translation().x() - branch);
    state.setVariablePosition(names[2], pose.translation().z());
    return true;
  }

  bool plan(const CartesianIKSolver & solver, bool enabled = true,
    const CancelFunction & canceled = [] {return false;},
    PlanningDeadline deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10),
    const planning_scene::PlanningSceneConstPtr & strict = nullptr)
  {
    config.cartesian_ik_recovery_enabled = enabled;
    return planCartesianMotion(*start, target, scene, config, output, *end, error,
      canceled, deadline, 0.0, {strict}, solver);
  }

  void unchanged() const
  {
    EXPECT_EQ(output.joint_trajectory.joint_names, std::vector<std::string>{"sentinel"});
    EXPECT_DOUBLE_EQ(end->getVariablePosition("left_q0"), 0.123);
  }

  void obstacle(double x)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base"; object.id = "released_box";
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {0.02, 0.02, 0.02};
    object.primitives.push_back(shape);
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0; pose.position.x = x;
    object.primitive_poses.push_back(pose);
    scene->processCollisionObjectMsg(object);
  }
};

TEST(CartesianIKRecovery, SubdivisionRecoversWithoutChangingTCPPath)
{
  RecoveryFixture f;
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto & limits, double timeout) {
      ++calls;
      EXPECT_GT(timeout, 0.0); EXPECT_LE(timeout, 0.1);
      // This fixture only converges from the unperturbed predecessor seed.
      for (double limit : limits) {if (limit > f.config.max_joint_step + 1e-9) {return false;}}
      if (pose.translation().x() - state.getGlobalLinkTransform(arm->getLinkModelNames().back()).translation().x() > 0.006)
      {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  ASSERT_TRUE(f.plan(solver)) << f.error;
  EXPECT_GT(calls, 8);
  EXPECT_EQ(f.output.joint_trajectory.points.size(), 9U);
  EXPECT_NEAR(f.end->getGlobalLinkTransform("left2").translation().x(), 0.04, 1e-9);
  for (const auto & point : f.output.joint_trajectory.points) {
    EXPECT_NEAR(point.positions[0], 0.0, 1e-9);
    EXPECT_NEAR(point.positions[2], 0.0, 1e-9);
    EXPECT_NEAR(point.positions[1], point.positions[4], 1e-9);
  }
}

TEST(CartesianIKRecovery, EarlierAlternativeBranchEscapesDeadEnd)
{
  RecoveryFixture f;
  auto solver = [](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      const double seed = state.getVariablePosition(arm->getVariableNames()[0]);
      if (arm->getName() == "left" && pose.translation().x() > 0.015 && std::abs(seed) < 0.02)
      {return false;}
      // Perturbed seeds select another branch only at the first waypoint.
      const double branch = std::abs(seed) < 1e-9 ? 0.0 : 0.1;
      return RecoveryFixture::solve(state, arm, pose, branch);
    };
  ASSERT_TRUE(f.plan(solver)) << f.error;
  EXPECT_NEAR(f.end->getVariablePosition("left_q0"), 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(f.output.joint_trajectory.points.front().positions[0], 0.0);
}

TEST(CartesianIKRecovery, LongPathRecoversFromTransientIKFailure)
{
  for (double length : {0.64, 0.65, 1.0}) {
    SCOPED_TRACE(length);
    RecoveryFixture f;
    f.target.left.translation().x() = f.target.right.translation().x() = length;
    int calls = 0;
    auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto &, double) {
        if (++calls == 1) {return false;}
        return RecoveryFixture::solve(state, arm, pose);
      };
    ASSERT_TRUE(f.plan(solver)) << f.error;
    EXPECT_GT(calls, 1);
    EXPECT_TRUE(f.end->getGlobalLinkTransform("left2").isApprox(f.target.left, 1e-9));
    EXPECT_TRUE(f.end->getGlobalLinkTransform("right2").isApprox(f.target.right, 1e-9));
  }
}

TEST(CartesianIKRecovery, LongPathCanSubdivide)
{
  for (double length : {0.32, 0.33, 0.65}) {
    SCOPED_TRACE(length);
    RecoveryFixture f;
    f.target.left.translation().x() = f.target.right.translation().x() = length;
    auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto & limits, double) {
        // Only the unperturbed seed with half-size steps can converge.
        for (double limit : limits) {
          if (limit > f.config.max_joint_step + 1e-9) {return false;}
        }
        const double previous_x = state.getGlobalLinkTransform(
          arm->getLinkModelNames().back()).translation().x();
        if (pose.translation().x() - previous_x > 0.006) {return false;}
        return RecoveryFixture::solve(state, arm, pose);
      };
    ASSERT_TRUE(f.plan(solver)) << f.error;
    EXPECT_EQ(f.output.joint_trajectory.points.size(),
      2U * static_cast<std::size_t>(std::ceil(length / f.config.cartesian_step)) + 1U);
    for (const auto & point : f.output.joint_trajectory.points) {
      EXPECT_NEAR(point.positions[0], 0.0, 1e-9);
      EXPECT_NEAR(point.positions[1], point.positions[4], 1e-9);
      EXPECT_NEAR(point.positions[2], 0.0, 1e-9);
    }
    EXPECT_TRUE(f.end->getGlobalLinkTransform("left2").isApprox(f.target.left, 1e-9));
    EXPECT_TRUE(f.end->getGlobalLinkTransform("right2").isApprox(f.target.right, 1e-9));
  }
}

TEST(CartesianIKRecovery, LongPathRetainsEarlyAlternativeBranch)
{
  for (double length : {0.32, 0.33, 0.65}) {
    SCOPED_TRACE(length);
    RecoveryFixture f;
    f.target.left.translation().x() = f.target.right.translation().x() = length;
    auto solver = [](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto &, double) {
        const double seed = state.getVariablePosition(arm->getVariableNames()[0]);
        if (arm->getName() == "left" && pose.translation().x() > 0.015 && std::abs(seed) < 0.02)
        {return false;}
        return RecoveryFixture::solve(state, arm, pose, std::abs(seed) < 1e-9 ? 0.0 : 0.1);
      };
    ASSERT_TRUE(f.plan(solver)) << f.error;
    EXPECT_NEAR(f.end->getVariablePosition("left_q0"), 0.1, 1e-9);
    EXPECT_DOUBLE_EQ(f.output.joint_trajectory.points.front().positions[0], 0.0);
    EXPECT_TRUE(f.end->getGlobalLinkTransform("left2").isApprox(f.target.left, 1e-9));
    EXPECT_TRUE(f.end->getGlobalLinkTransform("right2").isApprox(f.target.right, 1e-9));
  }
}

class CartesianSegmentRecovery : public ::testing::TestWithParam<std::string> {};

TEST_P(CartesianSegmentRecovery, DefaultFlagRecoversAndReportsSegment)
{
  RecoveryFixture f;
  const auto segment = GetParam();
  if (segment == "approach") {
    // Independent inward motions do not impose a rigid two-hand transform.
    f.target.right.translation().x() = -0.04;
  } else if (segment != "retreat") {
    f.target.left.translation().x() = f.target.right.translation().x() = 0.0;
    f.target.left.translation().z() = f.target.right.translation().z() =
      segment == "place_descent" ? -0.04 : 0.04;
  }
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  bool recovered = false;
  std::string report;
  ASSERT_TRUE(f.config.cartesian_ik_recovery_enabled);
  ASSERT_TRUE(planCartesianMotion(*f.start, f.target, f.scene, f.config, f.output, *f.end,
      f.error, [] {return false;}, std::chrono::steady_clock::now() + std::chrono::seconds(10),
      0.0, {nullptr, [&](bool success, const std::string & detail) {
          recovered = success; report = detail;
        }, segment}, solver)) << f.error;
  EXPECT_TRUE(recovered);
  EXPECT_NE(report.find("segment=" + segment), std::string::npos);
  EXPECT_NE(report.find("original: Cartesian dual-arm IK failed"), std::string::npos);
  EXPECT_TRUE(f.end->getGlobalLinkTransform("left2").isApprox(f.target.left, 1e-9));
  EXPECT_TRUE(f.end->getGlobalLinkTransform("right2").isApprox(f.target.right, 1e-9));
}

INSTANTIATE_TEST_SUITE_P(AllSharedSegments, CartesianSegmentRecovery,
  ::testing::Values("approach", "pick_lift", "lift_after_translation", "place_descent", "retreat"));

TEST(CartesianIKRecovery, DefaultPolicyUsesGlobalFlag)
{
  for (bool enabled : {false, true}) {
    RecoveryFixture f;
    f.config.cartesian_ik_recovery_enabled = enabled;
    int calls = 0;
    auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto &, double) {
        if (++calls == 1) {return false;}
        return RecoveryFixture::solve(state, arm, pose);
      };
    EXPECT_EQ(planCartesianMotion(*f.start, f.target, f.scene, f.config, f.output, *f.end,
        f.error, [] {return false;}, std::chrono::steady_clock::now() + std::chrono::seconds(10),
        0.0, {}, solver), enabled) << f.error;
    if (!enabled) {EXPECT_EQ(calls, 1); f.unchanged();}
  }
}

TEST(CartesianIKRecovery, NonRetreatMayFinishInAllowedBoxContact)
{
  RecoveryFixture f;
  f.obstacle(0.04);
  f.scene->getAllowedCollisionMatrixNonConst().setEntry("left2", "released_box", true);
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  ASSERT_TRUE(f.plan(solver)) << f.error;
  auto strict = planning_scene::PlanningScene::clone(f.scene);
  strict->getAllowedCollisionMatrixNonConst().setEntry("left2", "released_box", false);
  EXPECT_TRUE(strict->isStateColliding(*f.end, "dual"));
}

TEST(CartesianIKRecovery, AttachedBoxCannotCrossEnvironmentObstacle)
{
  RecoveryFixture f;
  f.obstacle(0.03);
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "left2";
  attached.object.header.frame_id = "left2";
  attached.object.id = "carried_box";
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {0.01, 0.01, 0.01};
  attached.object.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.z = 0.1;
  attached.object.primitive_poses.push_back(pose);
  attached.touch_links = {"left2", "right2"};
  ASSERT_TRUE(f.scene->processAttachedCollisionObjectMsg(attached));
  // Move the obstacle above the bare TCPs: only the attached box can hit it.
  auto obstacle = f.scene->getWorld()->getObject("released_box");
  Eigen::Isometry3d obstacle_pose = obstacle->shape_poses_[0];
  obstacle_pose.translation().z() = 0.1;
  ASSERT_TRUE(f.scene->getWorldNonConst()->moveShapeInObject(
      "released_box", obstacle->shapes_[0], obstacle_pose));
  EXPECT_FALSE(f.scene->isStateColliding(f.scene->getCurrentState(), "dual"));
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & target, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, target);
    };
  EXPECT_FALSE(f.plan(solver));
  EXPECT_EQ(f.error.find("collision=0"), std::string::npos) << f.error;
  f.unchanged();
}

TEST(CartesianIKRecovery, SuccessfulFastPathSkipsRecovery)
{
  RecoveryFixture f;
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      ++calls;
      return RecoveryFixture::solve(state, arm, pose);
    };
  ASSERT_TRUE(f.plan(solver)) << f.error;
  EXPECT_EQ(calls, 8);
  EXPECT_EQ(f.output.joint_trajectory.points.size(), 5U);
}

TEST(CartesianIKRecovery, CollisionValidationFailureDoesNotTriggerIKSearch)
{
  RecoveryFixture f;
  f.obstacle(0.02);
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      ++calls;
      return RecoveryFixture::solve(state, arm, pose);
    };
  EXPECT_FALSE(f.plan(solver));
  EXPECT_EQ(calls, 8);
  f.unchanged();
}

TEST(CartesianIKRecovery, RecoveryPreservesConfiguredJointMargin)
{
  RecoveryFixture f;
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  EXPECT_FALSE(planCartesianMotion(*f.start, f.target, f.scene, f.config, f.output, *f.end,
      f.error, [] {return false;}, std::chrono::steady_clock::now() + std::chrono::seconds(10),
      1.999, {}, solver));
  EXPECT_NE(f.error.find("margin="), std::string::npos) << f.error;
  EXPECT_EQ(f.error.find("margin=0"), std::string::npos) << f.error;
  f.unchanged();
}

TEST(CartesianIKRecovery, DisabledRecoveryReportsFailedArmAndPreservesOutput)
{
  RecoveryFixture f;
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      ++calls;
      return arm->getName() == "right" ? false : RecoveryFixture::solve(state, arm, pose);
    };
  EXPECT_FALSE(f.plan(solver, false));
  EXPECT_EQ(calls, 2);
  EXPECT_NE(f.error.find("arm=right"), std::string::npos);
  EXPECT_NE(f.error.find("target_xyzw="), std::string::npos);
  f.unchanged();
}

TEST(CartesianIKRecovery, RejectsUnsafeAlternativeSolutions)
{
  for (const auto & rejection : {"bounds", "continuity", "pose", "collision"}) {
    RecoveryFixture f;
    if (std::string(rejection) == "collision") {f.obstacle(0.02);}
    int calls = 0;
    auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto &, double) {
        if (++calls == 1) {return false;}
        const std::string reason(rejection);
        RecoveryFixture::solve(state, arm, pose, reason == "bounds" ? 3.0 :
          reason == "continuity" ? 0.5 : 0.0);
        if (reason == "pose") {state.setVariablePosition(arm->getVariableNames()[2], 0.2);}
        return true;
      };
    EXPECT_FALSE(f.plan(solver)) << rejection;
    EXPECT_NE(f.error.find("no Cartesian solution found within limits"), std::string::npos) << f.error;
    EXPECT_NE(f.error.find(std::string(rejection) + "="), std::string::npos) << f.error;
    EXPECT_EQ(f.error.find(std::string(rejection) + "=0"), std::string::npos) << f.error;
    f.unchanged();
  }
}

TEST(CartesianIKRecovery, AllowsInitialContactButRequiresClearEndpoint)
{
  RecoveryFixture f;
  f.obstacle(0.0);
  auto strict = planning_scene::PlanningScene::clone(f.scene);
  f.scene->getAllowedCollisionMatrixNonConst().setEntry("left2", "released_box", true);
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  ASSERT_TRUE(f.plan(solver, true, [] {return false;},
      std::chrono::steady_clock::now() + std::chrono::seconds(10), strict)) << f.error;
  EXPECT_TRUE(strict->isStateColliding(*f.start, "dual"));
  EXPECT_FALSE(strict->isStateColliding(*f.end, "dual"));
}

TEST(CartesianIKRecovery, RejectsEndpointStillTouchingReleasedBox)
{
  RecoveryFixture f;
  f.obstacle(0.04);
  auto strict = planning_scene::PlanningScene::clone(f.scene);
  f.scene->getAllowedCollisionMatrixNonConst().setEntry("left2", "released_box", true);
  int calls = 0;
  auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
      const auto &, double) {
      if (++calls == 1) {return false;}
      return RecoveryFixture::solve(state, arm, pose);
    };
  EXPECT_FALSE(f.plan(solver, true, [] {return false;},
      std::chrono::steady_clock::now() + std::chrono::seconds(10), strict));
  EXPECT_NE(f.error.find("endpoint has not cleared"), std::string::npos) << f.error;
  f.unchanged();
}

TEST(CartesianIKRecovery, CancellationAndDeadlineStopSearch)
{
  for (bool cancel : {false, true}) {
    RecoveryFixture f;
    int calls = 0;
    bool stopped = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    auto solver = [&](auto &, const auto *, const auto &, const auto &, const auto &, double) {
        if (++calls == 2) {stopped = true;}
        return false;
      };
    if (cancel) {
      EXPECT_FALSE(f.plan(solver, true, [&] {return stopped;}));
      EXPECT_EQ(calls, 2);
      EXPECT_NE(f.error.find("canceled"), std::string::npos);
    } else {
      EXPECT_FALSE(f.plan(solver, true, [] {return false;}, deadline - std::chrono::seconds(20)));
      EXPECT_EQ(calls, 0);
      EXPECT_NE(f.error.find("deadline"), std::string::npos);
    }
    f.unchanged();
  }
}

TEST(CartesianIKRecovery, BoundsTotalSearchCallsAndRejectsPartialPlans)
{
  for (double length : {0.04, 0.65}) {
    SCOPED_TRACE(length);
    RecoveryFixture f;
    f.target.left.translation().x() = f.target.right.translation().x() = length;
    int calls = 0;
    auto solver = [&](auto & state, const auto * arm, const auto & pose, const auto &,
        const auto &, double) {
        ++calls;
        if (pose.translation().x() > length - 0.001) {return false;}
        // Keep distinct branches so the test exercises the beam and call limit.
        return RecoveryFixture::solve(state, arm, pose,
          state.getVariablePosition(arm->getVariableNames()[0]));
      };
    EXPECT_FALSE(f.plan(solver));
    const int steps = static_cast<int>(std::ceil(length / f.config.cartesian_step));
    // Two arms and four branches at the 1x/2x/4x/8x resolutions, plus 128
    // extra calls per pass and the initial single-path traversal.
    EXPECT_LE(calls, 2 * steps + 8 * steps * (1 + 2 + 4 + 8) + 4 * 128);
    EXPECT_GT(calls, 2 * steps);
    EXPECT_NE(f.error.find("original:"), std::string::npos);
    f.unchanged();
  }
}

TEST(CartesianIKRecovery, DeadlineExpiringInsideRecoveryIKPreservesOutput)
{
  RecoveryFixture f;
  int calls = 0;
  auto solver = [&](auto &, const auto *, const auto &, const auto &, const auto &, double) {
      if (++calls == 2) {std::this_thread::sleep_for(std::chrono::milliseconds(125));}
      return false;
    };
  EXPECT_FALSE(f.plan(solver, true, [] {return false;},
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100)));
  EXPECT_EQ(calls, 2);
  EXPECT_NE(f.error.find("deadline exhausted"), std::string::npos);
  f.unchanged();
}
}  // namespace
}  // namespace agibot_x2_manipulation
