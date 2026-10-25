#include "criterion_planners.hpp"
#include "search/cell_search.hpp"

namespace mecanumbot_path_planner::detail
{
std::vector<std::size_t> findSafestPath(const GridProblem & problem)
{
  return findCellPath(problem, [&problem](std::size_t current, const Neighbor & neighbor) {
      const double average_risk =
        (problem.risk(current) + problem.risk(neighbor.cell)) / 2.0;
      return neighbor.distance *
             (1.0 + problem.options.safety_weight * average_risk);
    });
}
}  // namespace mecanumbot_path_planner::detail
