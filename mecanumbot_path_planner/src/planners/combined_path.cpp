#include "criterion_planners.hpp"

#include "search/cell_search.hpp"
#include "search/directional_search.hpp"

namespace mecanumbot_path_planner::detail
{
namespace
{
double combinedEdgeCost(
  const GridProblem & problem, std::size_t current, const Neighbor & neighbor)
{
  const double average_risk =
    (problem.risk(current) + problem.risk(neighbor.cell)) / 2.0;
  return neighbor.distance *
         (1.0 +
         problem.options.safetyWeight() * average_risk);
}
}  // namespace

std::vector<std::size_t> findCombinedPath(const GridProblem & problem)
{
  const auto edge_cost = [&problem](std::size_t current, const Neighbor & neighbor) {
      return combinedEdgeCost(problem, current, neighbor);
    };
  if (problem.options.smoothness_preference == 0.0) {
    return findCellPath(problem, edge_cost);
  }
  return findDirectionalPath(problem, edge_cost);
}
}  // namespace mecanumbot_path_planner::detail
