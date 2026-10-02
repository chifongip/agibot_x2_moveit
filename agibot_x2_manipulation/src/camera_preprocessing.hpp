#pragma once

#include <algorithm>
#include <cmath>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stdexcept>
#include <vector>

namespace agibot_x2_manipulation
{
inline cv::Size jpeg_size(const std::vector<uint8_t> & data)
{
  if (data.size() < 4 || data[0] != 0xff || data[1] != 0xd8) {
    throw std::runtime_error("Expected a JPEG image");
  }
  size_t i = 2;
  while (i < data.size()) {
    if (data[i++] != 0xff) {
      throw std::runtime_error("Invalid JPEG marker");
    }
    while (i < data.size() && data[i] == 0xff) {
      ++i;
    }
    if (i >= data.size()) {
      break;
    }
    const uint8_t marker = data[i++];
    if (marker == 0xda || marker == 0xd9) {
      break;
    }
    if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd8)) {
      continue;
    }
    if (i + 2 > data.size()) {
      break;
    }
    const size_t length = (static_cast<size_t>(data[i]) << 8) | data[i + 1];
    if (length < 2 || length > data.size() - i) {
      break;
    }
    if (marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 && marker != 0xc8 && marker != 0xcc) {
      if (length < 8) {
        break;
      }
      const int height = (data[i + 3] << 8) | data[i + 4];
      const int width = (data[i + 5] << 8) | data[i + 6];
      if (width > 0 && height > 0) {
        return {width, height};
      }
      break;
    }
    i += length;
  }
  throw std::runtime_error("JPEG dimensions missing or truncated");
}

inline cv::Size bounded_size(cv::Size source, int width, int height)
{
  if (source.width <= 0 || source.height <= 0 || width <= 0 || height <= 0) {
    throw std::invalid_argument("Image dimensions must be positive");
  }
  const double scale =
      std::min({1.0, double(width) / source.width, double(height) / source.height});
  return {std::max(1, int(std::floor(source.width * scale))),
          std::max(1, int(std::floor(source.height * scale)))};
}

inline int decode_factor(cv::Size source, cv::Size target)
{
  for (const int factor : {8, 4, 2}) {
    if ((source.width + factor - 1) / factor >= target.width &&
        (source.height + factor - 1) / factor >= target.height) {
      return factor;
    }
  }
  return 1;
}

inline cv::Mat decode_gray(const std::vector<uint8_t> & payload, int factor)
{
  std::vector<uint8_t> repaired;
  const auto * encoded = &payload;
  if (payload.size() >= 2 && (payload[payload.size() - 2] != 0xff || payload.back() != 0xd9)) {
    repaired = payload;
    repaired.insert(repaired.end(), {0xff, 0xd9});
    encoded = &repaired;
  }
  int flag = cv::IMREAD_GRAYSCALE;
  if (factor == 2) {
    flag = cv::IMREAD_REDUCED_GRAYSCALE_2;
  }
  if (factor == 4) {
    flag = cv::IMREAD_REDUCED_GRAYSCALE_4;
  }
  if (factor == 8) {
    flag = cv::IMREAD_REDUCED_GRAYSCALE_8;
  }
  auto image = cv::imdecode(*encoded, flag | cv::IMREAD_IGNORE_ORIENTATION);
  if (image.empty()) {
    throw std::runtime_error("JPEG decode failed");
  }
  return image;
}

class CameraGeometry
{
public:
  sensor_msgs::msg::CameraInfo configure(const sensor_msgs::msg::CameraInfo & info, cv::Size source,
                                         cv::Size target)
  {
    auto key = info;
    key.header = std_msgs::msg::Header();
    if (configured_ && key == key_ && source == source_ && target == target_) {
      return output_;
    }
    // A failed rebuild must never leave the old key pointing to changed maps.
    configured_ = false;
    const bool fisheye = info.distortion_model == "equidistant";
    if (!fisheye && info.distortion_model != "plumb_bob" &&
        info.distortion_model != "rational_polynomial") {
      throw std::runtime_error("Unsupported distortion model");
    }
    if ((fisheye && info.d.size() != 4) ||
        (info.distortion_model == "plumb_bob" && info.d.size() != 5) ||
        (info.distortion_model == "rational_polynomial" && info.d.size() != 8)) {
      throw std::runtime_error("Invalid distortion coefficient count");
    }
    if (info.width == 0 || info.height == 0 || info.k[0] <= 0 || info.k[4] <= 0 || info.p[0] <= 0 ||
        info.p[5] <= 0) {
      throw std::runtime_error("Invalid camera calibration");
    }
    for (const auto & values : {std::vector<double>(info.k.begin(), info.k.end()),
                                std::vector<double>(info.p.begin(), info.p.end()),
                                std::vector<double>(info.r.begin(), info.r.end()), info.d}) {
      for (const double value : values) {
        if (!std::isfinite(value)) {
          throw std::runtime_error("Non-finite calibration");
        }
      }
    }
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info);
    if (model.reducedResolution() != source) {
      throw std::runtime_error("JPEG dimensions disagree with calibration ROI/binning");
    }
    cv::Mat k(model.intrinsicMatrix(), true);
    cv::Mat projection(model.projectionMatrix(), true);
    const double sx = double(target.width) / source.width;
    const double sy = double(target.height) / source.height;
    k.row(0) *= sx;
    k.row(1) *= sy;
    projection.row(0) *= sx;
    projection.row(1) *= sy;
    cv::Mat r(3, 3, CV_64F, const_cast<double *>(info.r.data()));
    if (std::abs(cv::determinant(r) - 1.0) > 1e-3 ||
        cv::norm(r * r.t() - cv::Mat::eye(3, 3, CV_64F)) > 1e-3) {
      throw std::runtime_error("Invalid rectification rotation");
    }
    cv::Mat new_k = projection(cv::Rect(0, 0, 3, 3)).clone();
    identity_ = std::all_of(info.d.begin(), info.d.end(), [](double v) { return v == 0.0; }) &&
                cv::norm(r - cv::Mat::eye(3, 3, CV_64F)) < 1e-12 && cv::norm(k - new_k) < 1e-12;
    if (!identity_) {
      cv::Mat d(info.d, true);
      if (fisheye) {
        if (info.d.size() != 4) {
          throw std::runtime_error("Fisheye needs four coefficients");
        }
        cv::fisheye::initUndistortRectifyMap(k, d, r, new_k, target, CV_16SC2, map1_, map2_);
      } else {
        cv::initUndistortRectifyMap(k, d, r, new_k, target, CV_16SC2, map1_, map2_);
      }
    }
    output_ = sensor_msgs::msg::CameraInfo();
    output_.width = target.width;
    output_.height = target.height;
    output_.distortion_model = "plumb_bob";
    output_.d.assign(5, 0.0);
    output_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::copy(new_k.ptr<double>(), new_k.ptr<double>() + 9, output_.k.begin());
    std::copy(projection.ptr<double>(), projection.ptr<double>() + 12, output_.p.begin());
    key_ = key;
    source_ = source;
    target_ = target;
    configured_ = true;
    ++map_generation_;
    return output_;
  }

  cv::Mat rectify(const cv::Mat & image) const
  {
    if (identity_) {
      return image;
    }
    cv::Mat output;
    cv::remap(image, output, map1_, map2_, cv::INTER_LINEAR);
    return output;
  }
  size_t map_generation() const
  {
    return map_generation_;
  }

private:
  bool configured_{false}, identity_{false};
  size_t map_generation_{0};
  cv::Size source_, target_;
  cv::Mat map1_, map2_;
  sensor_msgs::msg::CameraInfo key_, output_;
};
} // namespace agibot_x2_manipulation
