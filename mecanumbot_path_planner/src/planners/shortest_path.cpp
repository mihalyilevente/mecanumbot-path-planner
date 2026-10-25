#include "criterion_planners.hpp"
#include "search/cell_search.hpp"

namespace mecanumbot_path_planner::detail
{
std::vector<std::size_t> findShortestPath(const GridProblem & problem)
{
  return findCellPath(problem, [](std::size_t, const Neighbor & neighbor) {
      return neighbor.distance;
    });
}
}  // namespace mecanumbot_path_planner::detail
