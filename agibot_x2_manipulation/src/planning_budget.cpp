#include "agibot_x2_manipulation/planning_budget.hpp"

#include <algorithm>
#include <cmath>

namespace agibot_x2_manipulation
{

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

double routeShareTimeout(double remaining_budget, std::size_t remaining_routes)
{
  if (!std::isfinite(remaining_budget) || remaining_budget <= 0.0 || remaining_routes == 0U) {
    return 0.0;
  }
  return remaining_budget / static_cast<double>(remaining_routes);
}

}  // namespace agibot_x2_manipulation
