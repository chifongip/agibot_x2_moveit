#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>

namespace agibot_x2_manipulation
{

enum class ResetAcquireResult
{
  ACQUIRED,
  CANCELED,
  TIMEOUT
};

class ResetCoordinator
{
public:
  bool reserveOperation();
  void releaseOperation();
  bool requestReset();
  ResetAcquireResult waitForResetAccess(
    std::chrono::nanoseconds timeout, const std::function<bool()> & canceled);
  void finishReset(bool successful);
  void notify();
  bool resetRequested() const;
  bool resetPending() const;

private:
  std::atomic<bool> busy_{false};
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> reset_pending_{false};
  std::atomic<bool> reset_owns_operation_{false};
  std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace agibot_x2_manipulation
