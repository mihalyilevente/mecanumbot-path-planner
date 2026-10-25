#ifndef MECANUMBOT_PATH_PLANNER__DIRECTIONAL_SEARCH_HPP_
#define MECANUMBOT_PATH_PLANNER__DIRECTIONAL_SEARCH_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "core/grid_problem.hpp"

namespace mecanumbot_path_planner::detail
{
using DirectionalEdgeCost = std::function<double(std::size_t, const Neighbor &)>;

std::vector<std::size_t> findDirectionalPath(
  const GridProblem & problem, const DirectionalEdgeCost & edge_cost);
}  // namespace mecanumbot_path_planner::detail

#endif  // MECANUMBOT_PATH_PLANNER__DIRECTIONAL_SEARCH_HPP_
