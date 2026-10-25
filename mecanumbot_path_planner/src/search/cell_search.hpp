#ifndef MECANUMBOT_PATH_PLANNER__CELL_SEARCH_HPP_
#define MECANUMBOT_PATH_PLANNER__CELL_SEARCH_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "core/grid_problem.hpp"

namespace mecanumbot_path_planner::detail
{

using EdgeCost = std::function<double(std::size_t, const Neighbor &)>;

std::vector<std::size_t> findCellPath(
  const GridProblem & problem, const EdgeCost & edge_cost);

}  // namespace mecanumbot_path_planner::detail

#endif  // MECANUMBOT_PATH_PLANNER__CELL_SEARCH_HPP_
