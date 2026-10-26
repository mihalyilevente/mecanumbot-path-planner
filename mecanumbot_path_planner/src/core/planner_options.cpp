#include "mecanumbot_path_planner/planner_options.hpp"

#include <cmath>
#include <stdexcept>

namespace mecanumbot_path_planner
{

OptimizationMode parseOptimizationMode(const std::string & mode)
{
  if (mode == "shortest") {
    return OptimizationMode::Shortest;
  }
  if (mode == "safest") {
    return OptimizationMode::Safest;
  }
  if (mode == "smoothest") {
    return OptimizationMode::Smoothest;
  }
  if (mode == "combined") {
    return OptimizationMode::Combined;
  }
  throw std::invalid_argument(
          "optimization_mode must be shortest, safest, smoothest or combined");
}

void validatePlannerOptions(const PlannerOptions & options)
{
  const bool valid_mode =
    options.mode == OptimizationMode::Shortest ||
    options.mode == OptimizationMode::Safest ||
    options.mode == OptimizationMode::Smoothest ||
    options.mode == OptimizationMode::Combined;
  if (!valid_mode) {
    throw std::invalid_argument("Invalid optimization mode");
  }

  const bool valid_preferences =
    std::isfinite(options.safety_preference) &&
    options.safety_preference >= 0.0 && options.safety_preference <= 1.0 &&
    std::isfinite(options.smoothness_preference) &&
    options.smoothness_preference >= 0.0 && options.smoothness_preference <= 1.0;
  if (!valid_preferences) {
    throw std::invalid_argument(
            "planning preferences must be finite values between 0 and 1");
  }
}

}  // namespace mecanumbot_path_planner
