#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>

namespace agibot_x2_manipulation
{

double adaptiveRetryTimeout(
  double total_budget, double elapsed, std::size_t remaining_candidates);

double endpointRouteTimeout(
  double remaining_budget, double endpoint_timeout, std::size_t endpoint_count);

double placeContinuationTimeout(
  double remaining_budget, double time_per_attempt, double return_timeout,
  std::size_t return_stage_count, bool has_alternative);

// Nested attempts shorten a deadline without canceling the enclosing search.
class ScopedPlanningDeadline
{
public:
  using Deadline = std::chrono::steady_clock::time_point;
  ScopedPlanningDeadline(Deadline & active, Deadline limit)
  : active_(active), previous_(active) {active_ = std::min(active, limit);}
  ~ScopedPlanningDeadline() {active_ = previous_;}
  ScopedPlanningDeadline(const ScopedPlanningDeadline &) = delete;
  ScopedPlanningDeadline & operator=(const ScopedPlanningDeadline &) = delete;
private:
  Deadline & active_;
  Deadline previous_;
};

// Reserve most of a search for fallback, including nested continuations.
double fastAttemptTimeout(double remaining_budget, double nominal_timeout);

}  // namespace agibot_x2_manipulation
