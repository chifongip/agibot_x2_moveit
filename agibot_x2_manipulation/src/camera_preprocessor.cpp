#include "camera_preprocessing.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

using namespace std::chrono_literals;
namespace agibot_x2_manipulation
{
class CameraPreprocessor : public rclcpp::Node
{
public:
  CameraPreprocessor() : Node("camera_preprocessor")
  {
    // Avoid OpenCV spawning a thread pool for each small selected image.
    cv::setNumThreads(1);
    const auto input = declare_parameter<std::string>("input_image_topic", "image/compressed");
    const auto calibration =
        declare_parameter<std::string>("input_camera_info_topic", "camera_info");
    const auto output = declare_parameter<std::string>("output_image_topic", "image_rect");
    const auto output_info =
        declare_parameter<std::string>("output_camera_info_topic", "rectified_camera_info");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    const double rate = declare_parameter<double>("max_rate_hz", 10.0);
    const auto reliability = declare_parameter<std::string>("input_reliability", "reliable");
    if (width_ <= 0 || height_ <= 0 || !std::isfinite(rate) || rate <= 0 ||
        (reliability != "reliable" && reliability != "best_effort") || input.empty() ||
        calibration.empty() || output.empty() || output_info.empty()) {
      throw std::invalid_argument("Invalid camera preprocessor parameters");
    }
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
    if (period_ <= std::chrono::steady_clock::duration::zero()) {
      throw std::invalid_argument("Processing rate exceeds clock resolution");
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();
    if (reliability == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(output, output_qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info, output_qos);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        // CameraInfo sensor QoS accepts both reliable and best-effort publishers.
        calibration, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
          std::lock_guard<std::mutex> lock(mutex_);
          calibration_ = info;
        });
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        input, qos, [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
          std::lock_guard<std::mutex> lock(mutex_);
          ++received_;
          if (latest_) {
            ++superseded_;
          }
          latest_ = image;
          ready_.notify_one();
        });
    report_timer_ = create_wall_timer(5s, [this]() {
      RCLCPP_INFO(get_logger(),
                  "received=%lu superseded=%lu processed=%lu failed=%lu last_processing_ms=%.3f",
                  received_.load(), superseded_.load(), processed_.load(), failed_.load(),
                  processing_ms_.load());
    });
    worker_ = std::thread([this]() { run(); });
  }

  ~CameraPreprocessor() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run()
  {
    auto next = std::chrono::steady_clock::now();
    while (true) {
      sensor_msgs::msg::CompressedImage::ConstSharedPtr image;
      sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this]() { return stopping_ || latest_; });
        if (stopping_) {
          return;
        }
        ready_.wait_until(lock, next, [this]() { return stopping_; });
        if (stopping_) {
          return;
        }
        image = std::move(latest_);
        info = calibration_;
      }
      const auto start = std::chrono::steady_clock::now();
      // Schedule from this start; never catch up missed intervals with a burst.
      next = start + period_;
      try {
        if (!info) {
          throw std::runtime_error("Waiting for CameraInfo");
        }
        if (image->header.frame_id.empty() || image->header.frame_id != info->header.frame_id) {
          throw std::runtime_error("Image and CameraInfo frame IDs must match");
        }
        const auto source = jpeg_size(image->data);
        const auto target = bounded_size(source, width_, height_);
        auto calibration = geometry_.configure(*info, source, target);
        auto gray = decode_gray(image->data, decode_factor(source, target));
        if (gray.size() != target) {
          cv::resize(gray, gray, target, 0, 0, cv::INTER_AREA);
        }
        gray = geometry_.rectify(gray);
        sensor_msgs::msg::Image output;
        output.header = image->header;
        output.width = target.width;
        output.height = target.height;
        output.encoding = "mono8";
        output.step = output.width;
        output.data.resize(output.step * output.height);
        for (int row = 0; row < gray.rows; ++row) {
          std::copy_n(gray.ptr<uint8_t>(row), gray.cols, output.data.begin() + row * output.step);
        }
        calibration.header = image->header;
        if (rclcpp::ok()) {
          info_pub_->publish(calibration);
          image_pub_->publish(std::move(output));
          ++processed_;
        }
      } catch (const std::exception & error) {
        ++failed_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping camera frame: %s",
                             error.what());
      }
      processing_ms_ =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count();
    }
  }

  int width_, height_;
  std::chrono::steady_clock::duration period_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_{false};
  std::thread worker_;
  CameraGeometry geometry_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_;
  std::atomic<unsigned long> received_{0}, superseded_{0}, processed_{0}, failed_{0};
  std::atomic<double> processing_ms_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
} // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<agibot_x2_manipulation::CameraPreprocessor>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_preprocessor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
