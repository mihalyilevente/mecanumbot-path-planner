#include "mecanumbot_path_planner/grid_a_star.hpp"

#include <cstddef>
#include <vector>

#include "planners/criterion_planners.hpp"
#include "grid_problem.hpp"

namespace mecanumbot_path_planner
{
namespace
{

std::vector<std::size_t> findCells(const detail::GridProblem & problem)
{
  switch (problem.options.mode) {
    case OptimizationMode::Shortest:
      return detail::findShortestPath(problem);
    case OptimizationMode::Safest:
      return problem.options.safety_weight == 0.0 ?
             detail::findShortestPath(problem) : detail::findSafestPath(problem);
    case OptimizationMode::Smoothest:
      return problem.options.smoothness_weight == 0.0 ?
             detail::findShortestPath(problem) : detail::findSmoothestPath(problem);
    case OptimizationMode::Combined:
      return problem.options.safety_weight == 0.0 &&
             problem.options.smoothness_weight == 0.0 ?
             detail::findShortestPath(problem) : detail::findCombinedPath(problem);
  }
  return {};  // validatePlannerOptions rejects unknown enum values.
}

}  // namespace

nav_msgs::msg::Path createGridPath(
  const nav2_costmap_2d::Costmap2D & map, const std::string & global_frame,
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerOptions & options)
{
  const detail::GridProblem problem(
    map, global_frame, stamp, start, goal, options);
  if (problem.start_cell == problem.goal_cell) {
    return detail::buildPath(problem, {});
  }
  return detail::buildPath(problem, findCells(problem));
}

nav_msgs::msg::Path createShortestPath(
  const nav2_costmap_2d::Costmap2D & map, const std::string & global_frame,
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, bool allow_unknown)
{
  PlannerOptions options;
  options.allow_unknown = allow_unknown;
  return createGridPath(map, global_frame, stamp, start, goal, options);
}

}  // namespace mecanumbot_path_planner
