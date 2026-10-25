#ifndef MECANUMBOT_PATH_PLANNER__SEARCH_QUEUE_HPP_
#define MECANUMBOT_PATH_PLANNER__SEARCH_QUEUE_HPP_

#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

namespace mecanumbot_path_planner::detail
{

struct QueueEntry
{
  double estimated_total;
  double cost_so_far;
  std::size_t state;
};

struct LowestScoreFirst
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const;
};

class SearchQueue
{
public:
  explicit SearchQueue(std::size_t state_count);

  void start(std::size_t state, double heuristic);
  bool empty() const;
  QueueEntry pop();
  bool isCurrent(const QueueEntry & entry) const;
  void relax(
    std::size_t from, std::size_t to, double candidate_cost,
    double heuristic);
  bool reached(std::size_t state) const;
  std::vector<std::size_t> trace(std::size_t end, std::size_t root) const;

private:
  std::vector<double> costs_;
  std::vector<std::size_t> parents_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowestScoreFirst> open_;
};

}  // namespace mecanumbot_path_planner::detail

#endif  // MECANUMBOT_PATH_PLANNER__SEARCH_QUEUE_HPP_
