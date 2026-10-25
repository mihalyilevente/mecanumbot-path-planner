#ifndef MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_
#define MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_

#include <string>

namespace mecanumbot_path_planner
{

enum class OptimizationMode {Shortest, Safest, Smoothest, Combined};

struct PlannerOptions
{
  OptimizationMode mode{OptimizationMode::Shortest};
  bool allow_unknown{false};
  double safety_weight{2.0};
  double smoothness_weight{0.5};
};

OptimizationMode parseOptimizationMode(const std::string & mode);
void validatePlannerOptions(const PlannerOptions & options);

}  // namespace mecanumbot_path_planner

#endif  // MECANUMBOT_PATH_PLANNER__PLANNER_OPTIONS_HPP_
