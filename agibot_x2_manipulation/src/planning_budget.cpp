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

}  // namespace agibot_x2_manipulation
