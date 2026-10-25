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

  const bool valid_weights =
    std::isfinite(options.safety_weight) && options.safety_weight >= 0.0 &&
    std::isfinite(options.smoothness_weight) && options.smoothness_weight >= 0.0;
  if (!valid_weights) {
    throw std::invalid_argument(
            "all planning weights must be finite and nonnegative");
  }
}

}  // namespace mecanumbot_path_planner
