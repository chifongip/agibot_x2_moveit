#pragma once

#include "pick_place/post_place_planner.hpp"

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace agibot_x2_manipulation
{

// Keep the remaining return sequence and its execution checkpoint together.
// A failed replan may change its candidate, never the live sequence. Once a
// segment is invalidated, Continue must replan before executing it again.
class PostPlaceProgress
{
public:
  explicit PostPlaceProgress(std::string prepare_target)
  : prepare_target_(std::move(prepare_target)) {}

  const PostPlaceSegment * current() const
  {
    return valid_ && index_ < plan_.segments.size() ? &plan_.segments[index_] : nullptr;
  }

  bool complete() const {return valid_ && !plan_.segments.empty() && index_ == plan_.segments.size();}
  bool include_retreat() const {return include_retreat_;}
  bool include_prepare() const {return include_prepare_;}
  void invalidate() {valid_ = false;}

  bool replan(const std::function<bool (PostPlacePlan &)> & planner, std::string & error)
  {
    invalidate();
    PostPlacePlan candidate;
    if (!planner(candidate)) {return false;}
    if (candidate.segments.empty()) {
      error = "empty return sequence";
      return false;
    }
    plan_ = std::move(candidate);
    index_ = 0;
    valid_ = true;
    return true;
  }

  void advance()
  {
    const auto * segment = current();
    if (!segment) {throw std::logic_error("return checkpoint has no validated segment");}
    if (segment->retreat) {include_retreat_ = false;}
    if (segment->name == "to_" + prepare_target_) {include_prepare_ = false;}
    ++index_;
  }

private:
  std::string prepare_target_;
  PostPlacePlan plan_;
  std::size_t index_{0};
  bool valid_{false};
  bool include_retreat_{true};
  bool include_prepare_{true};
};

}  // namespace agibot_x2_manipulation
