#include "directional_search.hpp"

#include <limits>

#include "nav2_core/exceptions.hpp"
#include "search_queue.hpp"

namespace mecanumbot_path_planner::detail
{
namespace
{
std::size_t checkedRootState(const GridProblem & problem)
{
  const auto count = static_cast<std::size_t>(problem.width) * problem.height;
  if (count > (std::numeric_limits<std::size_t>::max() - 2) /
    GridProblem::kDirectionCount)
  {
    throw nav2_core::PlannerException("Costmap has too many search states");
  }
  return count * GridProblem::kDirectionCount;
}

struct DirectionalStates
{
  explicit DirectionalStates(const GridProblem & problem)
  : root(checkedRootState(problem)), goal(root + 1) {}

  std::size_t forArrival(std::size_t cell, std::size_t direction) const
  {
    return cell * GridProblem::kDirectionCount + direction;
  }

  std::size_t cell(std::size_t state, std::size_t start_cell) const
  {
    return state == root ? start_cell : state / GridProblem::kDirectionCount;
  }

  std::size_t direction(std::size_t state) const
  {
    return state % GridProblem::kDirectionCount;
  }

  const std::size_t root;
  const std::size_t goal;
};

double arrivalTurnCost(
  const GridProblem & problem, const DirectionalStates & states,
  std::size_t current_state, std::size_t next_direction)
{
  if (current_state == states.root && !problem.has_start_connector) {
    return 0.0;
  }
  const double previous_angle = current_state == states.root ?
    problem.start_connector_angle :
    problem.directionAngle(states.direction(current_state));
  return problem.turnCost(previous_angle, problem.directionAngle(next_direction));
}

double finalConnectorTurnCost(
  const GridProblem & problem, const DirectionalStates & states,
  std::size_t arrival_state)
{
  if (!problem.has_goal_connector) {
    return 0.0;
  }
  return problem.turnCost(
    problem.directionAngle(states.direction(arrival_state)),
    problem.goal_connector_angle);
}

std::vector<std::size_t> reconstructCells(
  const GridProblem & problem, const DirectionalStates & states,
  const SearchQueue & search)
{
  const auto path_states = search.trace(states.goal, states.root);
  std::vector<std::size_t> cells{problem.start_cell};
  for (const auto state : path_states) {
    cells.push_back(states.cell(state, problem.start_cell));
  }
  return cells;
}
}  // namespace

std::vector<std::size_t> findDirectionalPath(
  const GridProblem & problem, const DirectionalEdgeCost & edge_cost)
{
  const DirectionalStates states(problem);
  SearchQueue search(states.goal + 1);
  search.start(states.root, problem.heuristic(problem.start_cell));

  while (!search.empty()) {
    const auto current = search.pop();
    if (!search.isCurrent(current)) {
      continue;
    }
    if (current.state == states.goal) {
      return reconstructCells(problem, states, search);
    }

    const auto current_cell = states.cell(current.state, problem.start_cell);
    if (current_cell == problem.goal_cell) {
      search.relax(
        current.state, states.goal,
        current.cost_so_far + finalConnectorTurnCost(problem, states, current.state),
        0.0);
    }

    for (const auto & neighbor : problem.neighbors(current_cell)) {
      const auto next_state = states.forArrival(neighbor.cell, neighbor.direction);
      const double turn = arrivalTurnCost(
        problem, states, current.state, neighbor.direction);
      search.relax(
        current.state, next_state,
        current.cost_so_far + edge_cost(current_cell, neighbor) + turn,
        problem.heuristic(neighbor.cell));
    }
  }
  throw nav2_core::PlannerException("No path connects start and goal on the costmap");
}
}  // namespace mecanumbot_path_planner::detail
