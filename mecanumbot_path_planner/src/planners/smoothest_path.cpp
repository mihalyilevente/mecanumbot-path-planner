#include "criterion_planners.hpp"

#include "search/directional_search.hpp"

namespace mecanumbot_path_planner::detail
{

std::vector<std::size_t> findSmoothestPath(const GridProblem & problem)
{
  return findDirectionalPath(
    problem,
    [](std::size_t, const Neighbor & neighbor) {
      return neighbor.distance;
    });
}

}  // namespace mecanumbot_path_planner::detail
