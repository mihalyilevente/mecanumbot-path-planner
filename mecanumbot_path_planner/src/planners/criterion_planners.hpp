#ifndef MECANUMBOT_PATH_PLANNER__CRITERION_PLANNERS_HPP_
#define MECANUMBOT_PATH_PLANNER__CRITERION_PLANNERS_HPP_

#include <cstddef>
#include <vector>

#include "core/grid_problem.hpp"

namespace mecanumbot_path_planner::detail
{

std::vector<std::size_t> findShortestPath(const GridProblem & problem);
std::vector<std::size_t> findSafestPath(const GridProblem & problem);
std::vector<std::size_t> findSmoothestPath(const GridProblem & problem);
std::vector<std::size_t> findCombinedPath(const GridProblem & problem);

}  // namespace mecanumbot_path_planner::detail

#endif  // MECANUMBOT_PATH_PLANNER__CRITERION_PLANNERS_HPP_
