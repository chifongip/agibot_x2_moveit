#include "camera_preprocessing.hpp"
#include <gtest/gtest.h>

using namespace agibot_x2_manipulation;

sensor_msgs::msg::CameraInfo calibration()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1280;
  info.height = 960;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {800, 0, 640, 0, 800, 480, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {800, 0, 640, 0, 0, 800, 480, 0, 0, 0, 1, 0};
  return info;
}

TEST(CameraPreprocessing, BoundsPreserveAspectAndNeverUpsample)
{
  EXPECT_EQ(bounded_size({1920, 1080}, 640, 480), cv::Size(640, 360));
  EXPECT_EQ(bounded_size({320, 240}, 640, 480), cv::Size(320, 240));
  EXPECT_THROW(bounded_size({1920, 1080}, 0, 480), std::invalid_argument);
  EXPECT_EQ(decode_factor({1920, 1080}, {640, 360}), 2);
  EXPECT_EQ(decode_factor({1280, 960}, {160, 120}), 8);
}

TEST(CameraPreprocessing, JpegDimensionsReductionAndMissingEndMarker)
{
  std::vector<uint8_t> data;
  cv::imencode(".jpg", cv::Mat(960, 1280, CV_8UC3, cv::Scalar(50, 50, 50)), data);
  EXPECT_EQ(jpeg_size(data), cv::Size(1280, 960));
  auto image = decode_gray(data, 4);
  EXPECT_EQ(image.size(), cv::Size(320, 240));
  EXPECT_EQ(image.type(), CV_8UC1);
  data.resize(data.size() - 2);
  EXPECT_FALSE(decode_gray(data, 2).empty());
  EXPECT_THROW(jpeg_size({0xff, 0xd8, 0xff, 0xc0, 0, 1}), std::runtime_error);
  EXPECT_THROW(jpeg_size({1, 2, 3, 4}), std::runtime_error);
}

TEST(CameraPreprocessing, CalibrationAndMapCache)
{
  CameraGeometry geometry;
  auto info = calibration();
  auto output = geometry.configure(info, {1280, 960}, {640, 480});
  EXPECT_DOUBLE_EQ(output.k[0], 400);
  EXPECT_DOUBLE_EQ(output.p[2], 320);
  EXPECT_EQ(output.width, 640u);
  EXPECT_EQ(output.binning_x, 0u);
  EXPECT_EQ(output.roi.width, 0u);
  EXPECT_EQ(geometry.map_generation(), 1u);
  info.header.frame_id = "camera";
  geometry.configure(info, {1280, 960}, {640, 480});
  EXPECT_EQ(geometry.map_generation(), 1u);
  info.d[0] = 0.1;
  geometry.configure(info, {1280, 960}, {640, 480});
  EXPECT_EQ(geometry.map_generation(), 2u);
  auto image = cv::Mat(480, 640, CV_8UC1, cv::Scalar(70));
  EXPECT_EQ(geometry.rectify(image).size(), image.size());
  EXPECT_THROW(geometry.configure(info, {640, 480}, {640, 480}), std::runtime_error);
  info.distortion_model = "unsupported";
  EXPECT_THROW(geometry.configure(info, {1280, 960}, {640, 480}), std::runtime_error);
}

TEST(CameraPreprocessing, BinningAndRoiNormalization)
{
  CameraGeometry geometry;
  auto info = calibration();
  info.binning_x = info.binning_y = 2;
  info.roi.x_offset = 160;
  info.roi.y_offset = 80;
  info.roi.width = 960;
  info.roi.height = 800;
  auto output = geometry.configure(info, {480, 400}, {240, 200});
  EXPECT_DOUBLE_EQ(output.k[0], 200);
  EXPECT_DOUBLE_EQ(output.k[2], 120);
  EXPECT_DOUBLE_EQ(output.k[5], 100);
}

TEST(CameraPreprocessing, FisheyeAndInvalidCalibration)
{
  auto info = calibration();
  CameraGeometry geometry;
  info.distortion_model = "equidistant";
  info.d = {0.01, 0, 0, 0};
  EXPECT_NO_THROW(geometry.configure(info, {1280, 960}, {640, 480}));
  info.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(geometry.configure(info, {1280, 960}, {640, 480}), std::runtime_error);
}
