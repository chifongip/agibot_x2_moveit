#include "pick_place/manipulation_state_store.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace agibot_x2_manipulation
{
namespace
{

class ManipulationStateStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = std::filesystem::temp_directory_path() /
      ("x2_manipulation_state_store_" + std::to_string(getpid()));
    std::filesystem::remove(path_);
    std::filesystem::remove(path_.string() + ".tmp");
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
    std::filesystem::remove(path_.string() + ".tmp");
  }

  std::filesystem::path path_;
};

TEST_F(ManipulationStateStoreTest, MissingAndLegacyStatesRemainCompatible)
{
  ManipulationStateStore store(path_.string());
  EXPECT_EQ(store.read().state, PersistedManipulationState::MISSING);

  {
    std::ofstream output(path_);
    output << "EMPTY\n";
  }
  EXPECT_EQ(store.read().state, PersistedManipulationState::EMPTY);

  {
    std::ofstream output(path_, std::ios::trunc);
    output << "HOLDING\n";
  }
  const auto holding = store.read();
  EXPECT_EQ(holding.state, PersistedManipulationState::HOLDING);
  EXPECT_FALSE(holding.held_object.valid);
}

TEST_F(ManipulationStateStoreTest, VersionTwoHoldingGeometryRoundTrips)
{
  ManipulationStateStore store(path_.string());
  PersistedHeldObject expected;
  expected.valid = true;
  expected.pose.translation() = Eigen::Vector3d(0.35, -0.02, 0.41);
  expected.pose.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.box_to_left_contact.translation() = Eigen::Vector3d(0.0, 0.16, 0.0);
  expected.box_to_right_contact.translation() = Eigen::Vector3d(0.0, -0.16, 0.0);

  store.write(PersistedManipulationState::HOLDING, expected);
  const auto actual = store.read();

  ASSERT_EQ(actual.state, PersistedManipulationState::HOLDING);
  ASSERT_TRUE(actual.held_object.valid);
  EXPECT_TRUE(actual.held_object.pose.matrix().isApprox(expected.pose.matrix(), 1e-12));
  EXPECT_TRUE(
    actual.held_object.box_to_left_contact.matrix().isApprox(
      expected.box_to_left_contact.matrix(), 1e-12));
  EXPECT_TRUE(
    actual.held_object.box_to_right_contact.matrix().isApprox(
      expected.box_to_right_contact.matrix(), 1e-12));
}

TEST_F(ManipulationStateStoreTest, IncompleteVersionTwoGeometryIsRejected)
{
  {
    std::ofstream output(path_);
    output << "VERSION 2\nSTATE HOLDING\nPOSE 0 0 0 0 0 0 1\nLEFT_CONTACT 0 0 0\n";
  }
  const auto record = ManipulationStateStore(path_.string()).read();
  EXPECT_EQ(record.state, PersistedManipulationState::HOLDING);
  EXPECT_FALSE(record.held_object.valid);
}

TEST_F(ManipulationStateStoreTest, EmptyStateDoesNotPersistHeldGeometry)
{
  PersistedHeldObject held_object;
  held_object.valid = true;
  ManipulationStateStore store(path_.string());
  store.write(PersistedManipulationState::EMPTY, held_object);

  const auto record = store.read();
  EXPECT_EQ(record.state, PersistedManipulationState::EMPTY);
  EXPECT_FALSE(record.held_object.valid);
}

}  // namespace
}  // namespace agibot_x2_manipulation
