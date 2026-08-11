#include "agibot_x2_manipulation/reset_coordinator.hpp"

namespace agibot_x2_manipulation
{

bool ResetCoordinator::reserveOperation()
{
  if (reset_requested_.load() || reset_pending_.load()) {
    return false;
  }
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    return false;
  }
  if (reset_requested_.load() || reset_pending_.load()) {
    releaseOperation();
    return false;
  }
  return true;
}

void ResetCoordinator::releaseOperation()
{
  busy_.store(false);
  condition_.notify_all();
}

bool ResetCoordinator::requestReset()
{
  bool expected = false;
  if (!reset_pending_.compare_exchange_strong(expected, true)) {
    return false;
  }
  reset_requested_.store(true);
  condition_.notify_all();
  return true;
}

ResetAcquireResult ResetCoordinator::waitForResetAccess(
  std::chrono::nanoseconds timeout, const std::function<bool()> & canceled)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = condition_.wait_for(
    lock, timeout, [this, &canceled]() {return !busy_.load() || canceled();});
  if (canceled()) {
    return ResetAcquireResult::CANCELED;
  }
  if (!ready || busy_.load()) {
    return ResetAcquireResult::TIMEOUT;
  }
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    return ResetAcquireResult::TIMEOUT;
  }
  reset_owns_operation_.store(true);
  return ResetAcquireResult::ACQUIRED;
}

void ResetCoordinator::finishReset(bool successful)
{
  if (successful) {
    reset_requested_.store(false);
  }
  if (reset_owns_operation_.exchange(false)) {
    releaseOperation();
  }
  reset_pending_.store(false);
  condition_.notify_all();
}

void ResetCoordinator::notify()
{
  condition_.notify_all();
}

bool ResetCoordinator::resetRequested() const
{
  return reset_requested_.load();
}

bool ResetCoordinator::resetPending() const
{
  return reset_pending_.load();
}

}  // namespace agibot_x2_manipulation
