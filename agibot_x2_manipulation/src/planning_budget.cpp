#include "agibot_x2_manipulation/planning_budget.hpp"

#include <algorithm>
#include <cmath>

namespace agibot_x2_manipulation
{

double fastAttemptTimeout(double remaining_budget, double nominal_timeout)
{
  if (!std::isfinite(remaining_budget) || !std::isfinite(nominal_timeout) ||
    remaining_budget <= 0.0 || nominal_timeout <= 0.0)
  {
    return 0.0;
  }
  return std::min(0.2 * remaining_budget, nominal_timeout);
}

double adaptiveRetryTimeout(
  double total_budget, double elapsed, std::size_t remaining_candidates)
{
  if (!std::isfinite(total_budget) || !std::isfinite(elapsed) ||
    total_budget <= 0.0 || elapsed < 0.0 || remaining_candidates == 0U)
  {
    return 0.0;
  }
  return std::max(0.0, total_budget - elapsed) /
         static_cast<double>(remaining_candidates);
}

double endpointRouteTimeout(
  double remaining_budget, double endpoint_timeout, std::size_t endpoint_count)
{
  if (!std::isfinite(remaining_budget) || !std::isfinite(endpoint_timeout) ||
    remaining_budget <= 0.0 || endpoint_timeout <= 0.0 || endpoint_count == 0U)
  {
    return 0.0;
  }
  // Allow scene attachment/restoration round trips in addition to endpoint plans.
  return std::min(remaining_budget, 2.0 + endpoint_timeout * endpoint_count);
}

double placeContinuationTimeout(
  double remaining_budget, double time_per_attempt, double return_timeout,
  std::size_t return_stage_count, bool has_alternative)
{
  if (!std::isfinite(remaining_budget) || remaining_budget <= 0.0 ||
    !std::isfinite(return_timeout) || return_timeout <= 0.0) {return 0.0;}
  const double candidate_limit = endpointRouteTimeout(
    remaining_budget, time_per_attempt, return_stage_count);
  if (candidate_limit <= 0.0) {return 0.0;}
  const double limit = std::min(return_timeout, remaining_budget);
  // A sole or final endpoint must be allowed the configured return retries,
  // Cartesian recovery, and validation time. Reserve short slices only while
  // there are other placement poses to try.
  return has_alternative ? std::min({limit, candidate_limit, 0.5 * remaining_budget}) : limit;
}

}  // namespace agibot_x2_manipulation
