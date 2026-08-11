#pragma once

#include <cstddef>

namespace agibot_x2_manipulation
{

double adaptiveRetryTimeout(
  double total_budget, double elapsed, std::size_t remaining_candidates);

}  // namespace agibot_x2_manipulation
