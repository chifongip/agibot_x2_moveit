#include "pick_place/locomanipulation_posture_controller.hpp"

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <zmq.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{

PickPlaceConfig controllerConfig()
{
  PickPlaceConfig config;
  config.posture_zmq_enabled = true;
  config.posture_zmq_endpoint = "tcp://127.0.0.1:0";
  config.posture_zmq_publish_rate_hz = 100.0;
  config.posture_settle_timeout = 1.0;
  config.posture_settle_duration = 0.0;
  config.posture_settle_samples = 3;
  config.leg_state_topic = "/posture_test/leg";
  config.waist_state_topic = "/posture_test/waist";
  return config;
}

aimdk_msgs::msg::JointStateArray feedback(const std::string & name)
{
  aimdk_msgs::msg::JointStateArray message;
  aimdk_msgs::msg::JointState joint;
  joint.name = name;
  joint.position = 0.0;
  joint.velocity = 0.0;
  message.joints.push_back(joint);
  return message;
}

class LocomanipulationPostureControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(LocomanipulationPostureControllerTest, PublishesCompleteSetpointAndWaitsForFreshFeedback)
{
  auto controller_node = std::make_shared<rclcpp::Node>("posture_controller_test");
  auto feedback_node = std::make_shared<rclcpp::Node>("posture_feedback_test");
  const auto config = controllerConfig();
  auto leg_publisher = feedback_node->create_publisher<aimdk_msgs::msg::JointStateArray>(
    config.leg_state_topic, rclcpp::SensorDataQoS());
  auto waist_publisher = feedback_node->create_publisher<aimdk_msgs::msg::JointStateArray>(
    config.waist_state_topic, rclcpp::SensorDataQoS());
  auto controller = std::make_unique<LocomanipulationPostureController>(controller_node, config);
  EXPECT_FALSE(controller->targetActive());

  void * subscriber_context = zmq_ctx_new();
  ASSERT_NE(subscriber_context, nullptr);
  void * subscriber = zmq_socket(subscriber_context, ZMQ_SUB);
  ASSERT_NE(subscriber, nullptr);
  const char subscription[] = "";
  ASSERT_EQ(
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, subscription, sizeof(subscription) - 1), 0);
  ASSERT_EQ(zmq_connect(subscriber, controller->endpoint().c_str()), 0);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(controller_node);
  executor.add_node(feedback_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  std::atomic<bool> publish_feedback{true};
  std::thread feedback_thread([&]() {
      while (publish_feedback.load()) {
        leg_publisher->publish(feedback("left_hip_pitch_joint"));
        waist_publisher->publish(feedback("waist_yaw_joint"));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  zmq_pollitem_t item{subscriber, 0, ZMQ_POLLIN, 0};
  EXPECT_EQ(zmq_poll(&item, 1, 50), 0);

  std::string error;
  EXPECT_TRUE(controller->applyAndWait(
      LocomanipulationPostureController::Target{0.50, -0.20}, []() {return false;}, error)) <<
    error;

  ASSERT_GT(zmq_poll(&item, 1, 1000), 0);
  char payload[128] = {};
  const int received = zmq_recv(subscriber, payload, sizeof(payload) - 1, 0);
  ASSERT_GT(received, 0);
  const std::string json(payload, static_cast<std::size_t>(received));
  EXPECT_NE(json.find("\"height\":0.5"), std::string::npos);
  EXPECT_NE(json.find("\"waist_yaw\":-0.2"), std::string::npos);
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '}');
  EXPECT_TRUE(controller->targetActive());
  EXPECT_TRUE(controller->deactivateTarget());
  EXPECT_FALSE(controller->targetActive());
  EXPECT_FALSE(controller->deactivateTarget());

  publish_feedback = false;
  feedback_thread.join();
  executor.cancel();
  spin_thread.join();
  zmq_close(subscriber);
  zmq_ctx_term(subscriber_context);
}

TEST_F(LocomanipulationPostureControllerTest, DisabledModeDoesNotBindOrWait)
{
  auto node = std::make_shared<rclcpp::Node>("posture_controller_disabled_test");
  auto config = controllerConfig();
  config.posture_zmq_enabled = false;
  config.posture_zmq_endpoint = "not-a-zmq-endpoint";
  LocomanipulationPostureController controller(node, config);

  std::string error;
  EXPECT_FALSE(controller.applyAndWait(
      LocomanipulationPostureController::Target{0.64, 0.0}, []() {return false;}, error));
  EXPECT_EQ(error, "locomanipulation posture ZMQ control is disabled");
  EXPECT_TRUE(controller.endpoint().empty());
  EXPECT_FALSE(controller.targetActive());
  EXPECT_FALSE(controller.deactivateTarget());
}

}  // namespace
}  // namespace agibot_x2_manipulation
