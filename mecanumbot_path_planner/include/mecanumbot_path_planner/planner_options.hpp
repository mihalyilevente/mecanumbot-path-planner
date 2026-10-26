#ifndef MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_
#define MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_

#include <string>

namespace mecanumbot_path_planner
{

enum class OptimizationMode {Shortest, Safest, Smoothest, Combined};

struct PlannerOptions
{
  static constexpr double kMaximumSafetyWeight = 2.0;
  static constexpr double kMaximumSmoothnessWeight = 0.5;

  OptimizationMode mode{OptimizationMode::Shortest};
  bool allow_unknown{false};
  double safety_preference{1.0};
  double smoothness_preference{1.0};

  double safetyWeight() const
  {
    return kMaximumSafetyWeight * safety_preference;
  }

  double smoothnessWeight() const
  {
    return kMaximumSmoothnessWeight * smoothness_preference;
  }
};

OptimizationMode parseOptimizationMode(const std::string & mode);
void validatePlannerOptions(const PlannerOptions & options);

}  // namespace mecanumbot_path_planner

#endif  // MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_
