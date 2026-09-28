#pragma once

#include <cstddef>

namespace agibot_x2_manipulation
{

double adaptiveRetryTimeout(
  double total_budget, double elapsed, std::size_t remaining_candidates);

double endpointRouteTimeout(
  double remaining_budget, double endpoint_timeout, std::size_t endpoint_count);

}  // namespace agibot_x2_manipulation
