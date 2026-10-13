#pragma once

#include <functional>
#include <string>

namespace agibot_x2_manipulation
{

// Keep correction pending across retry/Continue deadlines. The caller still
// owns feedback refresh, motion, contact validation, and physical release.
class ReleaseCheckpoint
{
public:
  using Attempt = std::function<bool (std::string &)>;
  using Interrupted = std::function<bool ()>;

  bool validate(const Attempt & check, const Attempt & correct,
    const Interrupted & interrupted, std::string & error)
  {
    const auto stopped = [&]() {
        if (!interrupted()) {return false;}
        if (error.empty()) {error = "release preflight canceled or deadline exhausted";}
        return true;
      };
    if (stopped()) {return false;}
    if (!correction_pending_) {
      if (check(error)) {return !stopped();}
      correction_pending_ = true;
    }
    // Do not overwrite the original exit failure with an attempt that cannot
    // start. The next retry or Continue will correct using its fresh budget.
    if (stopped() || !correct(error)) {return false;}
    correction_pending_ = false;
    if (stopped()) {return false;}
    if (!check(error)) {
      correction_pending_ = true;
      return false;
    }
    return !stopped();
  }

private:
  bool correction_pending_{false};
};

}  // namespace agibot_x2_manipulation
