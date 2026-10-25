#include "cell_search.hpp"

#include "nav2_core/exceptions.hpp"
#include "search_queue.hpp"

namespace mecanumbot_path_planner::detail
{

std::vector<std::size_t> findCellPath(
  const GridProblem & problem, const EdgeCost & edge_cost)
{
  const auto cell_count = static_cast<std::size_t>(problem.width) * problem.height;
  SearchQueue search(cell_count);
  search.start(problem.start_cell, problem.heuristic(problem.start_cell));

  while (!search.empty()) {
    const auto current = search.pop();
    if (!search.isCurrent(current)) {
      continue;
    }
    if (current.state == problem.goal_cell) {
      auto cells = search.trace(problem.goal_cell, problem.start_cell);
      cells.insert(cells.begin(), problem.start_cell);
      cells.push_back(problem.goal_cell);
      return cells;
    }

    for (const auto & neighbor : problem.neighbors(current.state)) {
      search.relax(
        current.state, neighbor.cell,
        current.cost_so_far + edge_cost(current.state, neighbor),
        problem.heuristic(neighbor.cell));
    }
  }

  throw nav2_core::PlannerException("No path connects start and goal on the costmap");
}

}  // namespace mecanumbot_path_planner::detail
