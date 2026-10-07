#include "pick_place/box_pose_tracker.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  if (!position.allFinite() || !rotation.coeffs().allFinite() || !std::isfinite(rotation.norm())) {
    throw std::invalid_argument("pose contains non-finite values");
  }
  if (rotation.norm() < 1e-9) {
    throw std::invalid_argument("pose quaternion has zero length");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = position;
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

}  // namespace

TableTagPoseStabilityFilter::TableTagPoseStabilityFilter(
  std::size_t stable_sample_count, double maximum_position_spread,
  double maximum_angular_spread, double maximum_sample_gap)
: stable_sample_count_(stable_sample_count),
  maximum_position_spread_(maximum_position_spread),
  maximum_angular_spread_(maximum_angular_spread),
  maximum_sample_gap_(rclcpp::Duration::from_seconds(maximum_sample_gap))
{
}

TableTagPoseStabilityUpdate TableTagPoseStabilityFilter::addSample(
  const Eigen::Isometry3d & sample, const rclcpp::Time & stamp)
{
  if (!sample.matrix().allFinite()) {
    return {};
  }
  if (!samples_.empty()) {
    const auto & previous_stamp = samples_.back().stamp;
    if (stamp <= previous_stamp) {
      return {};
    }
    if ((stamp - previous_stamp) > maximum_sample_gap_) {
      samples_.clear();
    }
  }

  samples_.push_back({sample, stamp});
  while (samples_.size() > stable_sample_count_) {
    samples_.pop_front();
  }
  if (samples_.size() != stable_sample_count_) {
    return {true, std::nullopt};
  }

  Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference(samples_.front().pose.linear());
  for (const auto & item : samples_) {
    mean_position += item.pose.translation();
    Eigen::Quaterniond quaternion(item.pose.linear());
    if (quaternion.dot(reference) < 0.0) {
      quaternion.coeffs() *= -1.0;
    }
    quaternion_sum += quaternion.coeffs();
  }
  if (quaternion_sum.norm() < 1e-9) {
    return {true, std::nullopt};
  }
  mean_position /= static_cast<double>(samples_.size());
  Eigen::Quaterniond mean_quaternion;
  mean_quaternion.coeffs() = quaternion_sum.normalized();

  double position_spread = 0.0;
  double angular_spread = 0.0;
  for (const auto & item : samples_) {
    position_spread = std::max(
      position_spread, (item.pose.translation() - mean_position).norm());
    const Eigen::Quaterniond quaternion(item.pose.linear());
    angular_spread = std::max(
      angular_spread, 2.0 * std::acos(
        std::clamp(std::abs(quaternion.dot(mean_quaternion)), 0.0, 1.0)));
  }
  if (position_spread > maximum_position_spread_ ||
    angular_spread > maximum_angular_spread_)
  {
    return {true, std::nullopt};
  }

  Eigen::Isometry3d stable_pose = Eigen::Isometry3d::Identity();
  stable_pose.translation() = mean_position;
  stable_pose.linear() = mean_quaternion.toRotationMatrix();
  return {true, stable_pose};
}

BoxPoseTracker::BoxPoseTracker(
  const rclcpp::Node::SharedPtr & node, std::string planning_frame,
  std::string legacy_topic, std::string states_topic, double maximum_age,
  double position_tolerance,
  double orientation_tolerance)
: node_(node), planning_frame_(std::move(planning_frame)), maximum_age_(maximum_age),
  position_tolerance_(position_tolerance), orientation_tolerance_(orientation_tolerance),
  tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
{
  legacy_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    std::move(legacy_topic), 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_poses_["legacy"] = TrackedBoxPose{"legacy", "", *message};
      pose_condition_.notify_all();
    });
  states_subscription_ = node_->create_subscription<
    agibot_x2_manipulation_msgs::msg::BoxStateArray>(
    std::move(states_topic), 10,
    [this](const agibot_x2_manipulation_msgs::msg::BoxStateArray::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & state : message->boxes) {
        if (state.instance_id.empty() || state.profile_id.empty()) {
          continue;
        }
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = state.header;
        pose.pose = state.pose;
        latest_poses_[state.instance_id] = TrackedBoxPose{
          state.instance_id, state.profile_id, std::move(pose)};
      }
      pose_condition_.notify_all();
    });
}

bool BoxPoseTracker::stablePose(
  const std::string & instance_id, TrackedBoxPose & pose,
  const std::optional<rclcpp::Time> & not_before) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto fresh = [this, &not_before](const TrackedBoxPose & candidate) {
      const rclcpp::Time stamp(candidate.pose.header.stamp, node_->get_clock()->get_clock_type());
      const double age = (node_->now() - stamp).seconds();
      return candidate.pose.header.frame_id == planning_frame_ && age >= 0.0 &&
             age <= maximum_age_ && (!not_before || stamp > *not_before);
    };
  if (!instance_id.empty()) {
    const auto found = latest_poses_.find(instance_id);
    if (found == latest_poses_.end() || !fresh(found->second)) {
      return false;
    }
    pose = found->second;
    return true;
  }

  const TrackedBoxPose * selected = nullptr;
  for (const auto & entry : latest_poses_) {
    const auto & candidate = entry.second;
    if (!fresh(candidate)) {
      continue;
    }
    if (selected) {
      return false;
    }
    selected = &candidate;
  }
  if (!selected) {
    return false;
  }
  pose = *selected;
  return true;
}

bool BoxPoseTracker::movedStablePose(
  const TrackedBoxPose & reference, TrackedBoxPose & latest, std::string & detail,
  const std::optional<rclcpp::Time> & not_before) const
{
  detail.clear();
  if (!stablePose(reference.instance_id, latest, not_before)) {return false;}
  bool moved = false;
  withinTolerance(reference, latest, detail, &moved);
  if (!moved) {detail.clear();}
  return moved;
}

bool BoxPoseTracker::waitForFresh(
  const std::function<bool(std::string &)> & ready, double timeout,
  const std::function<bool()> & canceled, const std::function<void()> & waiting,
  const std::string & description, std::string & error) const
{
  error.clear();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  bool announced = false;
  while (true) {
    if (canceled()) {
      error = "waiting for " + description + " canceled";
      return false;
    }
    if (ready(error)) {
      return true;
    }
    // A changed profile/pose is not a transient freshness failure.
    if (!error.empty()) {
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      error = "timed out waiting for " + description;
      return false;
    }
    if (!announced && waiting) {
      waiting();
      announced = true;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    pose_condition_.wait_for(lock, std::min(std::chrono::milliseconds(100),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
  }
}

bool BoxPoseTracker::waitForStablePose(
  const std::string & instance_id, double timeout, const std::function<bool()> & canceled,
  TrackedBoxPose & pose, std::string & error, const std::function<void()> & waiting,
  const std::optional<rclcpp::Time> & not_before) const
{
  return waitForFresh([&](std::string &) {return stablePose(instance_id, pose, not_before);},
    timeout, canceled, waiting, (instance_id.empty() ? "a uniquely selectable fresh box pose" :
    "fresh box pose for instance " + instance_id) +
    std::string(not_before ? " captured after the action request" : ""), error);
}

bool BoxPoseTracker::waitForUnchangedPoses(
  const std::vector<TrackedBoxPose> & references, double timeout,
  const std::function<bool()> & canceled, std::string & error,
  const std::function<void()> & waiting, bool * moved) const
{
  if (moved) {*moved = false;}
  // One deadline for the whole snapshot, rather than a timeout per missing tag.
  return waitForFresh([&](std::string & check_error) {
      const auto fresh = freshPoses();
      bool missing = false;
      for (const auto & reference : references) {
        const auto found = fresh.find(reference.instance_id);
        if (found == fresh.end()) {
          missing = true;
        } else if (!withinTolerance(reference, found->second, check_error, moved)) {
          check_error = "visible box instance '" + reference.instance_id +
            "' changed before motion: " + check_error;
          return false;
        }
      }
      return !missing;
    }, timeout, canceled, waiting, "fresh detections for the planned box snapshot", error);
}

std::map<std::string, TrackedBoxPose> BoxPoseTracker::freshPoses(
  const std::optional<rclcpp::Time> & not_before) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, TrackedBoxPose> result;
  for (const auto & entry : latest_poses_) {
    const auto & candidate = entry.second;
    const rclcpp::Time stamp(candidate.pose.header.stamp, node_->get_clock()->get_clock_type());
    const double age = (node_->now() - stamp).seconds();
    if (candidate.pose.header.frame_id == planning_frame_ && age >= 0.0 &&
      age <= maximum_age_ && (!not_before || stamp > *not_before))
    {
      result.emplace(entry.first, candidate);
    }
  }
  return result;
}

bool BoxPoseTracker::stillWithinTolerance(
  const TrackedBoxPose & reference, TrackedBoxPose & latest,
  std::string & error) const
{
  if (!stablePose(reference.instance_id, latest)) {
    error = "box pose became stale before approach";
    return false;
  }
  return withinTolerance(reference, latest, error);
}

bool BoxPoseTracker::withinTolerance(
  const TrackedBoxPose & reference, const TrackedBoxPose & latest, std::string & error,
  bool * moved) const
{
  if (latest.profile_id != reference.profile_id) {
    error = "box profile changed before approach";
    return false;
  }
  Eigen::Isometry3d current, reference_pose;
  try {
    current = toEigen(latest.pose.pose.pose);
    reference_pose = toEigen(reference.pose.pose.pose);
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
  const double position_error = (current.translation() - reference_pose.translation()).norm();
  const Eigen::Quaterniond reference_q(reference_pose.linear());
  const Eigen::Quaterniond current_q(current.linear());
  const double angular_error = 2.0 * std::acos(
    std::clamp(std::abs(reference_q.dot(current_q)), 0.0, 1.0));
  if (position_error > position_tolerance_ || angular_error > orientation_tolerance_) {
    if (moved) {*moved = true;}
    error = "box moved after planning: " + reference.instance_id +
      " (position=" + std::to_string(position_error) + " m, limit=" +
      std::to_string(position_tolerance_) + "; angle=" + std::to_string(angular_error) +
      " rad, limit=" + std::to_string(orientation_tolerance_) + ")";
    return false;
  }
  return true;
}

bool BoxPoseTracker::transformGoalPose(
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output,
  std::string & error) const
{
  if (input.header.frame_id.empty()) {
    error = "place_pose.frame_id is empty";
    return false;
  }
  try {
    if (input.header.frame_id == planning_frame_) {
      output = input;
    } else {
      const auto transform = tf_buffer_.lookupTransform(
        planning_frame_, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
    }
    output.header.frame_id = planning_frame_;
    return true;
  } catch (const tf2::TransformException & exception) {
    error = exception.what();
    return false;
  }
}

TableTagPoseTracker::TableTagPoseTracker(
  const rclcpp::Node::SharedPtr & node, std::string planning_frame,
  std::string tag_frame, std::string detections_topic, int tag_id,
  double minimum_decision_margin, std::size_t stable_sample_count, double maximum_age,
  double maximum_position_spread, double maximum_angular_spread,
  double maximum_sample_gap, StablePoseCallback stable_pose_callback)
: node_(node), planning_frame_(std::move(planning_frame)), tag_frame_(std::move(tag_frame)),
  tag_id_(tag_id), minimum_decision_margin_(minimum_decision_margin),
  maximum_age_(maximum_age), stability_filter_(
    stable_sample_count, maximum_position_spread, maximum_angular_spread, maximum_sample_gap),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_), stable_pose_callback_(std::move(stable_pose_callback))
{
  detections_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    std::move(detections_topic), rclcpp::SensorDataQoS(),
    std::bind(&TableTagPoseTracker::onDetections, this, std::placeholders::_1));
  pending_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20),
    std::bind(&TableTagPoseTracker::processPendingDetections, this));
}

void TableTagPoseTracker::onDetections(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message)
{
  const auto detection = std::find_if(message->detections.begin(), message->detections.end(),
    [this](const auto & item) {
      return item.id == tag_id_ && std::isfinite(item.decision_margin) &&
             item.decision_margin >= minimum_decision_margin_;
    });
  if (detection == message->detections.end()) {return;}
  const rclcpp::Time stamp(message->header.stamp);
  const double age = (node_->now() - stamp).seconds();
  if (stamp.nanoseconds() == 0 || age < 0.0 || age > maximum_age_) {return;}
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_detections_.size() >= 64) {pending_detections_.pop_front();}
    pending_detections_.push_back({message->header.stamp,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500)});
  }
  processPendingDetections();
}

void TableTagPoseTracker::processPendingDetections()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  while (!pending_detections_.empty()) {
    const auto pending = pending_detections_.front();
    const rclcpp::Time stamp(pending.stamp);
    const double age = (node_->now() - stamp).seconds();
    if (age < 0.0 || age > maximum_age_ ||
      std::chrono::steady_clock::now() >= pending.deadline)
    {
      pending_detections_.pop_front();
      continue;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(planning_frame_, tag_frame_, stamp);
      updateStablePose(tf2::transformToEigen(transform), pending.stamp);
      pending_detections_.pop_front();
    } catch (const tf2::TransformException &) {
      // Preserve sample order while waiting for detection-time TF on another topic.
      return;
    } catch (const std::exception & error) {
      pending_detections_.pop_front();
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Table tag pose rejected: %s", error.what());
    }
  }
}

void TableTagPoseTracker::updateStablePose(
  const Eigen::Isometry3d & sample, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped stable_pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto update = stability_filter_.addSample(sample, rclcpp::Time(stamp));
    if (!update.accepted_sample) {
      return;
    }
    have_stable_pose_ = false;
    if (!update.stable_pose) {
      return;
    }

    const Eigen::Isometry3d & stable_sample = *update.stable_pose;
    stable_pose_.header.frame_id = planning_frame_;
    stable_pose_.header.stamp = stamp;
    stable_pose_.pose.position.x = stable_sample.translation().x();
    stable_pose_.pose.position.y = stable_sample.translation().y();
    stable_pose_.pose.position.z = stable_sample.translation().z();
    stable_pose_.pose.orientation = tf2::toMsg(Eigen::Quaterniond(stable_sample.linear()));
    have_stable_pose_ = true;
    ++stable_generation_;
    stable_pose = stable_pose_;
  }
  stable_pose_condition_.notify_all();
  if (stable_pose_callback_) {
    stable_pose_callback_(stable_pose);
  }
}

bool TableTagPoseTracker::waitForStablePose(
  double timeout, const std::function<bool()> & canceled,
  geometry_msgs::msg::PoseStamped & output, std::string & error,
  const std::function<void()> & waiting, const std::optional<rclcpp::Time> & not_before) const
{
  std::uint64_t consumed_generation = 0;
  return waitForStablePoseAfter(0, timeout, canceled, output, consumed_generation, error, waiting, not_before);
}

std::uint64_t TableTagPoseTracker::generation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stable_generation_;
}

bool TableTagPoseTracker::waitForStablePoseAfter(
  std::uint64_t minimum_generation, double timeout, const std::function<bool()> & canceled,
  geometry_msgs::msg::PoseStamped & output, std::uint64_t & generation,
  std::string & error, const std::function<void()> & waiting,
  const std::optional<rclcpp::Time> & not_before) const
{
  error.clear();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  bool announced = false;
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (canceled()) {
      error = "waiting for stable table tag pose canceled";
      return false;
    }
    if (have_stable_pose_ && stable_generation_ > minimum_generation &&
      (!not_before || rclcpp::Time(stable_pose_.header.stamp,
        node_->get_clock()->get_clock_type()) > *not_before) &&
      (node_->now() - stable_pose_.header.stamp).seconds() >= 0.0 &&
      (node_->now() - stable_pose_.header.stamp).seconds() <= maximum_age_)
    {
      output = stable_pose_;
      generation = stable_generation_;
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      error = not_before ? "no fresh stable table tag pose captured after the action request" :
        "no fresh stable table tag pose";
      return false;
    }
    if (!announced && waiting) {
      lock.unlock();
      waiting();
      lock.lock();
      announced = true;
    }
    stable_pose_condition_.wait_for(
      lock, std::min(std::chrono::milliseconds(100),
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
  }
}

void BoxPoseTracker::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_poses_.clear();
}

}  // namespace agibot_x2_manipulation
