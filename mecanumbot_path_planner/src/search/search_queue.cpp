#include "search_queue.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/exceptions.hpp"

namespace mecanumbot_path_planner::detail
{

bool LowestScoreFirst::operator()(
  const QueueEntry & left, const QueueEntry & right) const
{
  if (left.estimated_total != right.estimated_total) {
    return left.estimated_total > right.estimated_total;
  }
  if (left.cost_so_far != right.cost_so_far) {
    return left.cost_so_far < right.cost_so_far;
  }
  return left.state > right.state;
}

SearchQueue::SearchQueue(std::size_t state_count)
: costs_(state_count, std::numeric_limits<double>::infinity()),
  parents_(state_count, state_count)
{
}

void SearchQueue::start(std::size_t state, double heuristic)
{
  costs_[state] = 0.0;
  open_.push({heuristic, 0.0, state});
}

bool SearchQueue::empty() const
{
  return open_.empty();
}

QueueEntry SearchQueue::pop()
{
  const auto result = open_.top();
  open_.pop();
  return result;
}

bool SearchQueue::isCurrent(const QueueEntry & entry) const
{
  return entry.cost_so_far == costs_[entry.state];
}

void SearchQueue::relax(
  std::size_t from, std::size_t to, double candidate_cost,
  double heuristic)
{
  if (!std::isfinite(candidate_cost + heuristic)) {
    throw nav2_core::PlannerException(
            "Planning cost overflow: reduce weights or map scale");
  }
  if (candidate_cost < costs_[to]) {
    costs_[to] = candidate_cost;
    parents_[to] = from;
    open_.push({candidate_cost + heuristic, candidate_cost, to});
  }
}

bool SearchQueue::reached(std::size_t state) const
{
  return std::isfinite(costs_[state]);
}

std::vector<std::size_t> SearchQueue::trace(
  std::size_t end, std::size_t root) const
{
  std::vector<std::size_t> result;
  for (auto state = parents_[end]; state != root; state = parents_[state]) {
    result.push_back(state);
  }
  std::reverse(result.begin(), result.end());
  return result;
}

}  // namespace mecanumbot_path_planner::detail
