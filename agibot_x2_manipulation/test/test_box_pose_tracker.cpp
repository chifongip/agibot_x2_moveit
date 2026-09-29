#include "pick_place/box_pose_tracker.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <memory>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{

class BoxPoseTrackerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("box_pose_wait_test");
    tracker_ = std::make_unique<BoxPoseTracker>(node_, "base_link",
      "/box_pose_wait_test/legacy", "/box_pose_wait_test/states", 0.25, 0.02, 0.1);
    publisher_ = node_->create_publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>(
      "/box_pose_wait_test/states", 10);
    executor_.add_node(node_);
    spinner_ = std::thread([this]() {executor_.spin();});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0U);
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
    executor_.remove_node(node_);
    tracker_.reset();
    publisher_.reset();
    node_.reset();
  }

  TrackedBoxPose pose(const std::string & instance, double x = 0.0)
  {
    TrackedBoxPose result;
    result.instance_id = instance;
    result.profile_id = "box";
    result.pose.header.frame_id = "base_link";
    result.pose.header.stamp = node_->now();
    result.pose.pose.pose.position.x = x;
    result.pose.pose.pose.orientation.w = 1.0;
    return result;
  }

  void publish(const std::vector<TrackedBoxPose> & boxes)
  {
    agibot_x2_manipulation_msgs::msg::BoxStateArray message;
    for (const auto & box : boxes) {
      agibot_x2_manipulation_msgs::msg::BoxState state;
      state.instance_id = box.instance_id;
      state.profile_id = box.profile_id;
      state.header = box.pose.header;
      state.pose = box.pose.pose;
      message.boxes.push_back(state);
    }
    publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<BoxPoseTracker> tracker_;
  rclcpp::Publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

TEST_F(BoxPoseTrackerTest, WaitsForFreshDetectionOfTheRequestedInstance)
{
  auto stale = pose("target");
  stale.pose.header.stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  publish({stale});
  TrackedBoxPose result;
  std::string error;
  int waiting = 0;
  EXPECT_TRUE(tracker_->waitForStablePose("target", 1.0, []() {return false;}, result, error,
      [&]() {
        ++waiting;
        // Another fresh instance must not be silently selected instead.
        publish({pose("other", 0.5), pose("target", 0.01)});
      })) << error;
  EXPECT_EQ(waiting, 1);
  EXPECT_EQ(result.instance_id, "target");
  EXPECT_NEAR(result.pose.pose.pose.position.x, 0.01, 1e-12);
  EXPECT_LE((node_->now() - result.pose.header.stamp).seconds(), 0.25);
}

TEST_F(BoxPoseTrackerTest, ResumesOnlyWhenAllPlannedBoxesAreFreshAndUnchanged)
{
  const std::vector<TrackedBoxPose> references{pose("first"), pose("second", 0.4)};
  std::string error;
  EXPECT_TRUE(tracker_->waitForUnchangedPoses(references, 1.0, []() {return false;}, error,
      [&]() {publish({pose("first", 0.01), pose("second", 0.41), pose("new", 1.0)});}))
    << error;
  // An empty snapshot (for example, excluding the held box) needs no detection.
  EXPECT_TRUE(tracker_->waitForUnchangedPoses({}, 0.0, []() {return false;}, error));
}

TEST_F(BoxPoseTrackerTest, DoesNotTreatAnExpiredObservationAsFreshDuringTheWait)
{
  auto stale = pose("target");
  stale.pose.header.stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  TrackedBoxPose result;
  std::string error;
  EXPECT_FALSE(tracker_->waitForStablePose("target", 0.08, []() {return false;}, result, error,
      [&]() {publish({stale});}));
  EXPECT_NE(error.find("timed out"), std::string::npos);
}

TEST_F(BoxPoseTrackerTest, RejectsMovementOrProfileChangesAfterReacquisition)
{
  const auto reference = pose("target");
  std::string error;
  bool moved = false;
  EXPECT_FALSE(tracker_->waitForUnchangedPoses({reference}, 1.0, []() {return false;}, error,
      [&]() {publish({pose("target", 0.1)}); }, &moved));
  EXPECT_TRUE(moved);
  EXPECT_NE(error.find("box moved after planning"), std::string::npos);
  TrackedBoxPose refreshed;
  ASSERT_TRUE(tracker_->waitForStablePose("target", 1.0, []() {return false;}, refreshed, error));
  EXPECT_TRUE(tracker_->waitForUnchangedPoses({refreshed}, 1.0, []() {return false;}, error,
      {}, &moved));
  EXPECT_FALSE(moved);
  tracker_->clear();
  EXPECT_FALSE(tracker_->waitForUnchangedPoses({reference}, 1.0, []() {return false;}, error,
      [&]() {
        auto changed = pose("target");
        changed.profile_id = "different_box";
        publish({changed});
      }, &moved));
  EXPECT_FALSE(moved);
  EXPECT_NE(error.find("profile changed"), std::string::npos);
}

TEST_F(BoxPoseTrackerTest, TimesOutOnceForTheEntireSnapshotAndHonorsCancellation)
{
  std::string error;
  const auto began = std::chrono::steady_clock::now();
  EXPECT_FALSE(tracker_->waitForUnchangedPoses(
      {pose("first"), pose("second"), pose("third")}, 0.08,
      []() {return false;}, error));
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - began).count();
  EXPECT_GE(elapsed, 0.07);
  EXPECT_LT(elapsed, 0.2);
  EXPECT_NE(error.find("timed out"), std::string::npos);
  bool canceled = false;
  TrackedBoxPose result;
  EXPECT_FALSE(tracker_->waitForStablePose("target", 10.0, [&]() {return canceled;}, result,
      error, [&]() {canceled = true;}));
  EXPECT_NE(error.find("canceled"), std::string::npos);
}

}  // namespace
}  // namespace agibot_x2_manipulation
