#include "pick_place/box_pose_tracker.hpp"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d poseAtX(double x)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x;
  return pose;
}

rclcpp::Time timeAt(int seconds)
{
  return rclcpp::Time(seconds, 0, RCL_ROS_TIME);
}

TEST(TableTagPoseStabilityFilter, RequiresThreeNewSamplesAfterAnOutage)
{
  TableTagPoseStabilityFilter filter(3, 0.005, 0.0523598776, 2.5);

  EXPECT_TRUE(filter.addSample(poseAtX(0.0), timeAt(1)).accepted_sample);
  EXPECT_TRUE(filter.addSample(poseAtX(0.0), timeAt(2)).accepted_sample);
  EXPECT_FALSE(filter.addSample(poseAtX(0.0), timeAt(2)).accepted_sample);

  const auto after_gap = filter.addSample(poseAtX(0.0), timeAt(10));
  EXPECT_TRUE(after_gap.accepted_sample);
  EXPECT_FALSE(after_gap.stable_pose);
  EXPECT_FALSE(filter.addSample(poseAtX(0.0), timeAt(11)).stable_pose);

  const auto stable = filter.addSample(poseAtX(0.0), timeAt(12));
  ASSERT_TRUE(stable.accepted_sample);
  ASSERT_TRUE(stable.stable_pose);
  EXPECT_NEAR(stable.stable_pose->translation().x(), 0.0, 1e-12);
}

TEST(TableTagPoseStabilityFilter, RejectsSamplesOutsideTheConfiguredSpread)
{
  TableTagPoseStabilityFilter filter(3, 0.005, 0.0523598776, 2.5);

  EXPECT_FALSE(filter.addSample(poseAtX(0.0), timeAt(1)).stable_pose);
  EXPECT_FALSE(filter.addSample(poseAtX(0.0), timeAt(2)).stable_pose);
  EXPECT_FALSE(filter.addSample(poseAtX(0.02), timeAt(3)).stable_pose);
}

class TableTagPoseTrackerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("table_tag_pose_tracker_test");
    createTracker(5.0);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    detections_publisher_ = node_->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/table_tag_test/detections", rclcpp::SensorDataQoS());
    executor_.add_node(node_);
    spinFor(std::chrono::milliseconds(50));
  }

  void TearDown() override
  {
    tracker_.reset();
    detections_publisher_.reset();
    transform_broadcaster_.reset();
    executor_.remove_node(node_);
    node_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void createTracker(double maximum_age)
  {
    tracker_ = std::make_unique<TableTagPlacePoseTracker>(
      node_, "base_link", "tag9", "/table_tag_test/detections", 9, 20.0,
      BoxDimensions{0.15, 0.32, 0.32}, 0.47, 0.0, 0.15, 0.0, 3, maximum_age,
      0.005, 0.0523598776, 2.5);
  }

  void publishTransform(double x, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "tag9";
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    transform_broadcaster_->sendTransform(transform);
  }

  void publishDetection(const rclcpp::Time & stamp)
  {
    apriltag_msgs::msg::AprilTagDetectionArray detections;
    detections.header.stamp = stamp;
    detections.header.frame_id = "front_center_camera";
    apriltag_msgs::msg::AprilTagDetection tag;
    tag.id = 9;
    tag.decision_margin = 30.0F;
    detections.detections.push_back(tag);
    detections_publisher_->publish(detections);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<TableTagPlacePoseTracker> tracker_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_publisher_;
};

class BoxPoseTrackerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("box_pose_tracker_test");
    tracker_ = std::make_unique<BoxPoseTracker>(
      node_, "base_link", "/box_pose_tracker_test/legacy",
      "/box_pose_tracker_test/states", 5.0, 0.01, 0.1);
    publisher_ = node_->create_publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>(
      "/box_pose_tracker_test/states", 10);
    executor_.add_node(node_);
    spinFor(std::chrono::milliseconds(50));
  }

  void TearDown() override
  {
    publisher_.reset();
    tracker_.reset();
    executor_.remove_node(node_);
    node_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  agibot_x2_manipulation_msgs::msg::BoxState box(
    const std::string & instance_id, const std::string & profile_id, double x) const
  {
    agibot_x2_manipulation_msgs::msg::BoxState result;
    result.header.frame_id = "base_link";
    result.header.stamp = node_->now();
    result.instance_id = instance_id;
    result.profile_id = profile_id;
    result.pose.pose.position.x = x;
    result.pose.pose.orientation.w = 1.0;
    return result;
  }

  void publish(const std::vector<agibot_x2_manipulation_msgs::msg::BoxState> & boxes)
  {
    agibot_x2_manipulation_msgs::msg::BoxStateArray states;
    states.header.frame_id = "base_link";
    states.header.stamp = node_->now();
    states.boxes = boxes;
    publisher_->publish(states);
    spinFor(std::chrono::milliseconds(50));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<BoxPoseTracker> tracker_;
  rclcpp::Publisher<agibot_x2_manipulation_msgs::msg::BoxStateArray>::SharedPtr publisher_;
};

TEST_F(BoxPoseTrackerTest, RetainsMultipleVisibleBoxesAndRejectsProfileChanges)
{
  publish({box("tag:0", "small_carton", 0.2), box("tag:180", "grey_box", 0.5)});

  const auto visible = tracker_->freshPoses();
  ASSERT_EQ(visible.size(), 2U);
  ASSERT_EQ(visible.at("tag:0").profile_id, "small_carton");
  ASSERT_EQ(visible.at("tag:180").profile_id, "grey_box");

  const auto reference = visible.at("tag:0");
  publish({box("tag:0", "grey_box", 0.2), box("tag:180", "grey_box", 0.5)});
  TrackedBoxPose latest;
  std::string error;
  EXPECT_FALSE(tracker_->stillWithinTolerance(reference, latest, error));
  EXPECT_EQ(error, "box profile changed before approach");
}

TEST_F(TableTagPoseTrackerTest, UsesLatestTransformWhenDetectionTfArrivesLater)
{
  const auto now = node_->now();
  const std::array transforms{
    now - rclcpp::Duration::from_seconds(0.30),
    now - rclcpp::Duration::from_seconds(0.20),
    now - rclcpp::Duration::from_seconds(0.10),
  };

  // Every detection is slightly newer than its latest available tag TF, which
  // models delivery on separate DDS topics. The robot is stationary, so the
  // latest transform remains the correct table-tag pose for each detection.
  for (const auto & transform_stamp : transforms) {
    publishTransform(0.25, transform_stamp);
    spinFor(std::chrono::milliseconds(10));
    publishDetection(transform_stamp + rclcpp::Duration::from_seconds(0.01));
    spinFor(std::chrono::milliseconds(20));
  }

  geometry_msgs::msg::PoseStamped stable_pose;
  std::string error;
  ASSERT_TRUE(tracker_->waitForStablePose(
      0.1, []() {return false;}, stable_pose, error)) << error;
  EXPECT_NEAR(stable_pose.pose.position.x, 0.25, 1e-6);
}

TEST_F(TableTagPoseTrackerTest, RejectsStaleLatestTransform)
{
  tracker_.reset();
  createTracker(0.05);
  const auto now = node_->now();
  publishTransform(0.25, now - rclcpp::Duration::from_seconds(0.10));
  spinFor(std::chrono::milliseconds(20));
  publishDetection(node_->now());
  spinFor(std::chrono::milliseconds(20));

  geometry_msgs::msg::PoseStamped stable_pose;
  std::string error;
  EXPECT_FALSE(tracker_->waitForStablePose(
      0.05, []() {return false;}, stable_pose, error));
  EXPECT_EQ(error, "no fresh stable table tag pose");
}

}  // namespace
}  // namespace agibot_x2_manipulation
