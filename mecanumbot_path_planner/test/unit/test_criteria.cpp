#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "mecanumbot_path_planner/grid_a_star.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace
{
using namespace mecanumbot_path_planner;
using nav2_costmap_2d::Costmap2D;
using nav2_core::PlannerException;

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}

nav_msgs::msg::Path plan(
  const Costmap2D & map, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const PlannerOptions & options)
{
  return createGridPath(map, "map", builtin_interfaces::msg::Time{}, start, goal, options);
}

struct Metrics
{
  double length{0.0};
  double exposure{0.0};
  double turns{0.0};
};

// Score the returned geometry independently of the search implementation.
Metrics measure(const Costmap2D & map, const nav_msgs::msg::Path & path)
{
  Metrics m;
  double previous_x = 0.0, previous_y = 0.0;
  auto risk = [&](const auto & p) {
      unsigned int x, y;
      if (!map.worldToMap(p.x, p.y, x, y)) {throw std::runtime_error("Path left map");}
      return std::min(1.0, map.getCost(x, y) / 252.0);
    };
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d = std::hypot(dx, dy);
    m.length += d;
    m.exposure += d * (risk(a) + risk(b)) / 2.0;
    if (d > 0.0) {
      if (previous_x != 0.0 || previous_y != 0.0) {
        const double angle = std::atan2(previous_x * dy - previous_y * dx,
            previous_x * dx + previous_y * dy);
        m.turns += angle * angle;
      }
      previous_x = dx;
      previous_y = dy;
    }
  }
  return m;
}

double score(const Metrics & m, const PlannerOptions & options)
{
  if (options.mode == OptimizationMode::Safest) {
    return m.length + options.safetyWeight() * m.exposure;
  }
  if (options.mode == OptimizationMode::Smoothest) {
    return m.length + options.smoothnessWeight() * m.turns;
  }
  if (options.mode == OptimizationMode::Combined) {
    return m.length + options.safetyWeight() * m.exposure +
           options.smoothnessWeight() * m.turns;
  }
  return m.length;
}

TEST(Criteria, SafestAcceptsLongerRouteForLessCostExposure)
{
  Costmap2D map(9, 5, 1.0, 0.0, 0.0);
  for (unsigned int x = 1; x < 8; ++x) {map.setCost(x, 2, 220);}
  const auto start = pose(0.5, 2.5), goal = pose(8.5, 2.5);
  PlannerOptions options;
  const auto shortest = measure(map, plan(map, start, goal, options));
  options.mode = OptimizationMode::Safest;
  const auto safest = measure(map, plan(map, start, goal, options));
  EXPECT_GT(safest.length, shortest.length);
  EXPECT_LT(safest.exposure, shortest.exposure);
  EXPECT_LT(score(safest, options), score(shortest, options));
  EXPECT_DOUBLE_EQ(safest.exposure, 0.0);
}

TEST(Criteria, SmoothestAcceptsLongerRouteWithFewerTurns)
{
  // Two one-cell-wide corridors: a length-10 zigzag (four 90-degree turns),
  // and a length-12 perimeter route (two 90-degree turns).
  Costmap2D map(9, 6, 1.0, 0.0, 0.0, 254);
  for (unsigned int x = 0; x <= 2; ++x) {map.setCost(x, 3, 0);}
  for (unsigned int x = 2; x <= 4; ++x) {map.setCost(x, 2, 0);}
  for (unsigned int x = 4; x <= 8; ++x) {map.setCost(x, 3, 0);}
  for (unsigned int y = 3; y <= 5; ++y) {
    map.setCost(0, y, 0);
    map.setCost(8, y, 0);
  }
  for (unsigned int x = 0; x <= 8; ++x) {map.setCost(x, 5, 0);}
  const auto start = pose(0.5, 3.5), goal = pose(8.5, 3.5);
  PlannerOptions options;
  const auto shortest = measure(map, plan(map, start, goal, options));
  options.mode = OptimizationMode::Smoothest;
  options.smoothness_preference = 1.0;
  const auto smoothest = measure(map, plan(map, start, goal, options));
  EXPECT_DOUBLE_EQ(shortest.length, 10.0);
  EXPECT_DOUBLE_EQ(smoothest.length, 12.0);
  EXPECT_LT(smoothest.turns, shortest.turns);
  EXPECT_LT(score(smoothest, options), score(shortest, options));
}

TEST(Criteria, ZeroPreferencesRestoreShortestPathExactly)
{
  Costmap2D map(9, 5, 0.3, 0.0, 0.0);
  for (unsigned int x = 1; x < 8; ++x) {map.setCost(x, 2, 220);}
  const auto start = pose(0.15, 0.15), goal = pose(2.55, 1.35);
  PlannerOptions options;
  const auto shortest = plan(map, start, goal, options);
  options.safety_preference = options.smoothness_preference = 0.0;
  for (const auto mode : {OptimizationMode::Safest, OptimizationMode::Smoothest,
    OptimizationMode::Combined})
  {
    options.mode = mode;
    EXPECT_EQ(plan(map, start, goal, options), shortest);
  }
}

TEST(Criteria, CombinedModeUsesSafetyAndSmoothnessPreferences)
{
  Costmap2D map(9, 5, 1.0, 0.0, 0.0);
  for (unsigned int x = 1; x < 8; ++x) {map.setCost(x, 2, 220);}
  const auto start = pose(0.5, 2.5), goal = pose(8.5, 2.5);
  PlannerOptions combined;
  combined.mode = OptimizationMode::Combined;

  const auto balanced = measure(map, plan(map, start, goal, combined));
  EXPECT_GT(balanced.length, 8.0);
  EXPECT_DOUBLE_EQ(balanced.exposure, 0.0);

  combined.safety_preference = 0.0;
  combined.smoothness_preference = 0.0;
  PlannerOptions shortest_options;
  EXPECT_EQ(plan(map, start, goal, combined), plan(map, start, goal, shortest_options));
}

TEST(Criteria, UnknownSpaceIsBlockedOrPenalizedRatherThanTreatedAsFree)
{
  Costmap2D map(7, 3, 1.0, 0.0, 0.0);
  for (unsigned int x = 1; x < 6; ++x) {map.setCost(x, 1, 255);}
  PlannerOptions options;
  options.mode = OptimizationMode::Safest;
  options.allow_unknown = true;
  auto path = plan(map, pose(0.5, 1.5), pose(6.5, 1.5), options);
  EXPECT_DOUBLE_EQ(measure(map, path).exposure, 0.0);
  Costmap2D corridor(3, 1, 1.0, 0.0, 0.0);
  corridor.setCost(1, 0, 255);
  path = plan(corridor, pose(0.5, 0.5), pose(2.5, 0.5), options);
  EXPECT_DOUBLE_EQ(measure(corridor, path).exposure, 1.0);
  options.allow_unknown = false;
  EXPECT_THROW(plan(corridor, pose(0.5, 0.5), pose(2.5, 0.5), options), PlannerException);
}

TEST(Criteria, AllModesKeepCollisionBoundsAndSameCellRules)
{
  for (const auto mode : {OptimizationMode::Shortest, OptimizationMode::Safest,
    OptimizationMode::Smoothest, OptimizationMode::Combined})
  {
    PlannerOptions options;
    options.mode = mode;
    Costmap2D map(2, 2, 1.0, 0.0, 0.0);
    map.setCost(1, 0, 253);
    map.setCost(0, 1, 254);
    EXPECT_THROW(plan(map, pose(0.5, 0.5), pose(1.5, 1.5), options), PlannerException);
    EXPECT_THROW(plan(map, pose(-0.1, 0.5), pose(1.5, 1.5), options), PlannerException);
    EXPECT_THROW(plan(map, pose(0.5, 0.5), pose(1.5, 0.5), options), PlannerException);
    const auto path = plan(map, pose(0.1, 0.1), pose(0.8, 0.8), options);
    ASSERT_EQ(path.poses.size(), 2u);
    EXPECT_EQ(path.poses.front().pose, pose(0.1, 0.1).pose);
    EXPECT_EQ(path.poses.back().pose, pose(0.8, 0.8).pose);
  }
}

TEST(Criteria, PreferencesScaleLinearlyToDefinedMaximumPenalties)
{
  PlannerOptions options;
  options.safety_preference = 0.0;
  options.smoothness_preference = 0.0;
  EXPECT_DOUBLE_EQ(options.safetyWeight(), 0.0);
  EXPECT_DOUBLE_EQ(options.smoothnessWeight(), 0.0);

  options.safety_preference = 0.5;
  options.smoothness_preference = 0.5;
  EXPECT_DOUBLE_EQ(
    options.safetyWeight(), PlannerOptions::kMaximumSafetyWeight / 2.0);
  EXPECT_DOUBLE_EQ(
    options.smoothnessWeight(), PlannerOptions::kMaximumSmoothnessWeight / 2.0);

  options.safety_preference = 1.0;
  options.smoothness_preference = 1.0;
  EXPECT_DOUBLE_EQ(options.safetyWeight(), PlannerOptions::kMaximumSafetyWeight);
  EXPECT_DOUBLE_EQ(options.smoothnessWeight(), PlannerOptions::kMaximumSmoothnessWeight);
}

TEST(Criteria, RejectsInvalidPreferencesAndModes)
{
  PlannerOptions options;
  for (double value : {-1.0, 1.000001, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    options.safety_preference = value;
    EXPECT_THROW(validatePlannerOptions(options), std::invalid_argument);
    options.safety_preference = 1.0;
    options.smoothness_preference = value;
    EXPECT_THROW(validatePlannerOptions(options), std::invalid_argument);
    options.smoothness_preference = 1.0;
  }
  options.mode = static_cast<OptimizationMode>(99);
  EXPECT_THROW(validatePlannerOptions(options), std::invalid_argument);
  EXPECT_THROW(parseOptimizationMode("invalid"), std::invalid_argument);
  options.mode = OptimizationMode::Combined;
  options.safety_preference = 0.0;
  options.smoothness_preference = 0.0;
  EXPECT_NO_THROW(validatePlannerOptions(options));
}

// Independent graph oracle. Uses a different direction order, explicit directed
// edges, dot/cross turn angles and dense Dijkstra without an A* heuristic.
double referenceScore(
  const Costmap2D & map, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const PlannerOptions & options)
{
  const int width = map.getSizeInCellsX(), height = map.getSizeInCellsY();
  const int count = width * height, root = count * 8, sink = root + 1;
  const int dxs[] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dys[] = {0, 1, 1, 1, 0, -1, -1, -1};
  const double res = map.getResolution();
  const double initial_x = res / 2.0 - start.pose.position.x;
  const double initial_y = res / 2.0 - start.pose.position.y;
  const double final_x = goal.pose.position.x - (width - 0.5) * res;
  const double final_y = goal.pose.position.y - (height - 0.5) * res;
  auto turn = [&](double ax, double ay, double bx, double by) {
      if ((options.mode != OptimizationMode::Smoothest &&
        options.mode != OptimizationMode::Combined) ||
        (ax == 0.0 && ay == 0.0) ||
        (bx == 0.0 && by == 0.0)) {return 0.0;}
      const double angle = std::atan2(ax * by - ay * bx, ax * bx + ay * by);
      return options.smoothnessWeight() * angle * angle;
    };
  auto travel = [&](int a, int b, double distance) {
      if (options.mode != OptimizationMode::Safest &&
        options.mode != OptimizationMode::Combined)
      {
        return distance;
      }
      const double risk_a = std::min(1.0, map.getCost(a % width, a / width) / 252.0);
      const double risk_b = std::min(1.0, map.getCost(b % width, b / width) / 252.0);
      return distance *
             (1.0 + options.safetyWeight() * (risk_a + risk_b) / 2.0);
    };
  auto free = [&](int x, int y) {return map.getCost(x, y) < 253;};
  struct Edge {int to; double cost;};
  std::vector<std::vector<Edge>> graph(sink + 1);
  for (int state = 0; state <= root; ++state) {
    const int cell = state == root ? 0 : state / 8;
    const int x = cell % width, y = cell / width;
    if (!free(x, y)) {continue;}
    const double previous_x = state == root ? initial_x : dxs[state % 8];
    const double previous_y = state == root ? initial_y : dys[state % 8];
    if (cell == count - 1) {
      graph[state].push_back({sink, travel(cell, cell, std::hypot(final_x, final_y)) +
        turn(previous_x, previous_y, final_x, final_y)});
    }
    for (int dir = 0; dir < 8; ++dir) {
      const int nx = x + dxs[dir], ny = y + dys[dir];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height || !free(nx, ny)) {continue;}
      if (nx != x && ny != y && (!free(nx, y) || !free(x, ny))) {continue;}
      const int next = ny * width + nx;
      const double cost = travel(cell, next, res * std::hypot(dxs[dir], dys[dir])) +
        turn(previous_x, previous_y, dxs[dir], dys[dir]);
      graph[state].push_back({next * 8 + dir, cost});
    }
  }
  std::vector<double> distances(sink + 1, std::numeric_limits<double>::infinity());
  std::vector<bool> used(sink + 1, false);
  distances[root] = travel(0, 0, std::hypot(initial_x, initial_y));
  for (int step = 0; step <= sink; ++step) {
    int best = -1;
    for (int i = 0; i <= sink; ++i) {
      if (!used[i] && (best < 0 || distances[i] < distances[best])) {best = i;}
    }
    if (best < 0 || !std::isfinite(distances[best]) || best == sink) {break;}
    used[best] = true;
    for (const auto & edge : graph[best]) {
      distances[edge.to] = std::min(distances[edge.to], distances[best] + edge.cost);
    }
  }
  return distances[sink];
}

TEST(Criteria, PreferenceObjectivesMatchIndependentOracleIncludingEndpointTurns)
{
  std::mt19937 random(108);
  for (int trial = 0; trial < 40; ++trial) {
    SCOPED_TRACE(trial);
    const double res = trial % 2 ? 0.25 : 1.0;
    Costmap2D map(5, 5, res, 0.0, 0.0);
    for (unsigned int y = 0; y < 5; ++y) {
      for (unsigned int x = 0; x < 5; ++x) {
        map.setCost(x, y, random() % 100 < 20 ? 254 : random() % 253);
      }
    }
    map.setCost(0, 0, 100);
    map.setCost(4, 4, 150);
    // Include centered endpoints and connectors with different travel directions.
    const double offset = trial % 3 == 0 ? 0.5 : (trial % 3 == 1 ? 0.2 : 0.8);
    const auto start = pose(offset * res, (1.0 - offset) * res);
    auto goal = pose((4.0 + offset) * res, (5.0 - offset) * res);
    for (const auto mode : {OptimizationMode::Safest, OptimizationMode::Smoothest,
      OptimizationMode::Combined})
    {
      PlannerOptions options;
      options.mode = mode;
      options.safety_preference = (trial % 4) / 3.0;
      options.smoothness_preference = (trial % 4) / 3.0;
      const double expected = referenceScore(map, start, goal, options);
      if (!std::isfinite(expected)) {
        EXPECT_THROW(plan(map, start, goal, options), PlannerException);
        continue;
      }
      const auto path = plan(map, start, goal, options);
      EXPECT_NEAR(score(measure(map, path), options), expected, 1e-9);
      EXPECT_EQ(path, plan(map, start, goal, options));
      const auto original_goal = goal;
      goal.pose.orientation.w = 0.0;
      goal.pose.orientation.z = 1.0;
      const auto yaw_changed = plan(map, start, goal, options);
      ASSERT_EQ(path.poses.size(), yaw_changed.poses.size());
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        EXPECT_EQ(path.poses[i].pose.position, yaw_changed.poses[i].pose.position);
      }
      goal = original_goal;
    }
  }
}

}  // namespace
