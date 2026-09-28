#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>

namespace agibot_x2_manipulation
{

struct PhaseRetryStatus
{
  std::string task_id;
  std::string action;
  std::string status{"idle"};
  std::string phase;
  std::string last_completed_phase;
  std::string object_disposition{"not_attached"};
  std::string failure;
  unsigned int pause_id{0};
  int attempt{0};
  int maximum_attempts{0};
  bool can_continue{false};
};

// One action worker owns run(); service callbacks only signal the same worker.
class PhaseRetryController
{
public:
  using Clock = std::chrono::steady_clock;
  using Cancel = std::function<bool ()>;
  using Attempt = std::function<bool (Clock::time_point, std::string &)>;
  using Publish = std::function<void (const PhaseRetryStatus &)>;

  void begin(const std::string & task_id, const std::string & action,
    const std::string & disposition)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = PhaseRetryStatus{};
    status_.task_id = task_id;
    status_.action = action;
    status_.object_disposition = disposition;
    status_.status = "running";
    continue_requested_ = false;
  }

  PhaseRetryStatus snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  void checkpoint(const std::string & phase, const std::string & disposition)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.last_completed_phase = phase;
    status_.object_disposition = disposition;
  }

  void finish(const std::string & status, const std::string & failure = "")
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.status = status;
    if (!failure.empty()) {status_.failure = failure;}
    status_.can_continue = false;
    continue_requested_ = false;
    condition_.notify_all();
  }

  bool requestContinue(const std::string & task_id, unsigned int pause_id,
    std::string & error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_id != status_.task_id || pause_id != status_.pause_id ||
      status_.status != "paused" || !status_.can_continue || continue_requested_)
    {
      error = "task is not paused at this checkpoint, or Continue was already accepted";
      return false;
    }
    continue_requested_ = true;
    status_.can_continue = false;
    condition_.notify_all();
    return true;
  }

  bool run(const std::string & phase, bool plan_only, int attempts,
    double timeout, double delay, const Attempt & attempt, const Cancel & canceled,
    const Publish & publish, std::string & error,
    const Cancel & terminal = []() {return false;})
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_.phase = phase;
      status_.maximum_attempts = attempts;
    }
    while (!canceled()) {
      const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(timeout));
      for (int index = 1; index <= attempts && !canceled() && Clock::now() < deadline; ++index) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          status_.attempt = index;
          status_.status = index == 1 ? "running" : "retrying";
          status_.can_continue = false;
        }
        publish(snapshot());
        error.clear();
        if (attempt(deadline, error)) {
          if (canceled()) {break;}
          {
            std::lock_guard<std::mutex> lock(mutex_);
            status_.last_completed_phase = phase;
            status_.status = "running";
            status_.failure.clear();
          }
          publish(snapshot());
          return true;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          status_.failure = error.empty() ? "phase failed without a diagnostic" : error;
        }
        publish(snapshot());
        if (terminal()) {return false;}
        if (index < attempts) {
          std::unique_lock<std::mutex> lock(mutex_);
          const auto next = std::min(deadline, Clock::now() +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(delay)));
          while (!canceled() && Clock::now() < next) {
            condition_.wait_for(lock, std::chrono::milliseconds(20));
          }
        }
      }
      if (canceled()) {break;}
      if (plan_only) {return false;}
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.status = "paused";
        status_.can_continue = true;
        ++status_.pause_id;
        continue_requested_ = false;
      }
      publish(snapshot());
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!continue_requested_ && !canceled()) {
          condition_.wait_for(lock, std::chrono::milliseconds(20));
        }
        continue_requested_ = false;
      }
    }
    error = "task canceled while running, retrying, or paused";
    finish("canceled");
    publish(snapshot());
    return false;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  PhaseRetryStatus status_;
  bool continue_requested_{false};
};

}  // namespace agibot_x2_manipulation
