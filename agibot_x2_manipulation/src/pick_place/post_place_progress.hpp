#pragma once

#include "pick_place/post_place_planner.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace agibot_x2_manipulation
{

enum class PostPlaceStage {RETREAT, PREPARE, READY, COMPLETE};

// Physical stage completion survives failed replans and Continue. Cached route
// segments belong only to the active stage and never authorize a later stage.
class PostPlaceProgress
{
public:
  explicit PostPlaceProgress(std::string prepare_target)
  : prepare_target_(std::move(prepare_target)) {}

  PostPlaceStage stage() const {return stage_;}
  bool complete() const {return stage_ == PostPlaceStage::COMPLETE;}
  const PostPlaceSegment * current() const
  {
    return valid_ && index_ < plan_.segments.size() ? &plan_.segments[index_] : nullptr;
  }
  bool stage_complete() const
  {
    return valid_ && !plan_.segments.empty() && index_ == plan_.segments.size();
  }
  void invalidate() {valid_ = false;}

  bool replan(const std::function<bool (PostPlacePlan &)> & planner, std::string & error)
  {
    invalidate();
    PostPlacePlan candidate;
    if (!planner(candidate)) {return false;}
    if (candidate.segments.empty()) {
      error = "empty post-place stage";
      return false;
    }
    plan_ = std::move(candidate);
    index_ = 0;
    valid_ = true;
    return true;
  }

  void advance_segment()
  {
    if (!current()) {throw std::logic_error("post-place segment has no validated plan");}
    ++index_;
  }

  void advance_stage()
  {
    if (!stage_complete()) {throw std::logic_error("post-place stage has not completed");}
    switch (stage_) {
      case PostPlaceStage::RETREAT:
        stage_ = prepare_target_.empty() ? PostPlaceStage::READY : PostPlaceStage::PREPARE;
        break;
      case PostPlaceStage::PREPARE: stage_ = PostPlaceStage::READY; break;
      case PostPlaceStage::READY: stage_ = PostPlaceStage::COMPLETE; break;
      case PostPlaceStage::COMPLETE: throw std::logic_error("post-place workflow already complete");
    }
    invalidate();
  }

private:
  std::string prepare_target_;
  PostPlaceStage stage_{PostPlaceStage::RETREAT};
  PostPlacePlan plan_;
  std::size_t index_{0};
  bool valid_{false};
};

}  // namespace agibot_x2_manipulation
