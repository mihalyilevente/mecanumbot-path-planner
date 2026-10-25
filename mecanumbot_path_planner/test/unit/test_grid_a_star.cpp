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
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_core::PlannerException;
using mecanumbot_path_planner::createShortestPath;

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

nav_msgs::msg::Path plan(
  const Costmap2D & map, double sx, double sy, double gx, double gy, bool unknown = false)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  return createShortestPath(map, "map", stamp, pose(sx, sy), pose(gx, gy), unknown);
}

double length(const nav_msgs::msg::Path & path)
{
  double result = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    result += std::hypot(b.x - a.x, b.y - a.y);
  }
  return result;
}

void expectValidCells(const Costmap2D & map, const nav_msgs::msg::Path & path)
{
  unsigned int px = 0, py = 0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    unsigned int x, y;
    const auto & p = path.poses[i].pose.position;
    ASSERT_TRUE(map.worldToMap(p.x, p.y, x, y));
    EXPECT_LT(map.getCost(x, y), INSCRIBED_INFLATED_OBSTACLE);
    if (i > 0) {
      EXPECT_LE(std::abs(static_cast<int>(x) - static_cast<int>(px)), 1);
      EXPECT_LE(std::abs(static_cast<int>(y) - static_cast<int>(py)), 1);
      if (x != px && y != py) {
        EXPECT_LT(map.getCost(x, py), INSCRIBED_INFLATED_OBSTACLE);
        EXPECT_LT(map.getCost(px, y), INSCRIBED_INFLATED_OBSTACLE);
      }
    }
    px = x;
    py = y;
  }
}

TEST(GridAStar, EmptyMapHasOptimalDiagonalAndCardinalLength)
{
  Costmap2D map(6, 5, 1.0, 0.0, 0.0);
  const auto path = plan(map, 0.5, 0.5, 5.5, 3.5);
  EXPECT_NEAR(length(path), 3.0 * std::sqrt(2.0) + 2.0, 1e-12);
  expectValidCells(map, path);
  EXPECT_DOUBLE_EQ(length(plan(map, 0.5, 2.5, 5.5, 2.5)), 5.0);
}

TEST(GridAStar, UsesOffsetOriginResolutionAndExactEndpoints)
{
  Costmap2D map(5, 4, 0.2, -3.0, 7.0);
  auto start = pose(-2.98, 7.03);
  auto goal = pose(-2.02, 7.75);
  goal.pose.orientation.z = 1.0;
  goal.pose.orientation.w = 0.0;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123;
  const auto path = createShortestPath(map, "map", stamp, start, goal);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_EQ(path.poses.front().pose, start.pose);
  EXPECT_EQ(path.poses.back().pose, goal.pose);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.header.stamp, stamp);
  for (const auto & p : path.poses) {
    EXPECT_EQ(p.header, path.header);
    const auto & q = p.pose.orientation;
    EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
  }
  expectValidCells(map, path);
}

TEST(GridAStar, RoutesThroughWallGapAndRejectsClosedWall)
{
  Costmap2D map(7, 7, 1.0, 0.0, 0.0);
  for (unsigned int y = 0; y < 6; ++y) {
    map.setCost(3, y, LETHAL_OBSTACLE);
  }
  const auto path = plan(map, 1.5, 1.5, 5.5, 1.5);
  expectValidCells(map, path);
  EXPECT_TRUE(std::any_of(path.poses.begin(), path.poses.end(), [](const auto & p) {
    return p.pose.position.x == 3.5 && p.pose.position.y == 6.5;
  }));
  map.setCost(3, 6, LETHAL_OBSTACLE);
  EXPECT_THROW(plan(map, 1.5, 1.5, 5.5, 1.5), PlannerException);
}

TEST(GridAStar, NeverCutsDiagonalCorners)
{
  Costmap2D map(2, 2, 1.0, 0.0, 0.0);
  map.setCost(1, 0, LETHAL_OBSTACLE);
  auto path = plan(map, 0.5, 0.5, 1.5, 1.5);
  EXPECT_DOUBLE_EQ(length(path), 2.0);
  expectValidCells(map, path);
  map.setCost(0, 1, LETHAL_OBSTACLE);
  EXPECT_THROW(plan(map, 0.5, 0.5, 1.5, 1.5), PlannerException);
}

TEST(GridAStar, UnknownSpaceIsOptInIncludingEndpointsAndDiagonalSides)
{
  Costmap2D map(3, 1, 1.0, 0.0, 0.0);
  map.setCost(1, 0, NO_INFORMATION);
  EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 0.5), PlannerException);
  EXPECT_DOUBLE_EQ(length(plan(map, 0.5, 0.5, 2.5, 0.5, true)), 2.0);
  map.setCost(0, 0, NO_INFORMATION);
  map.setCost(2, 0, NO_INFORMATION);
  EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 0.5), PlannerException);
  EXPECT_NO_THROW(plan(map, 0.5, 0.5, 2.5, 0.5, true));
  Costmap2D corner(2, 2, 1.0, 0.0, 0.0);
  corner.setCost(1, 0, NO_INFORMATION);
  EXPECT_DOUBLE_EQ(length(plan(corner, 0.5, 0.5, 1.5, 1.5)), 2.0);
  EXPECT_NEAR(length(plan(corner, 0.5, 0.5, 1.5, 1.5, true)), std::sqrt(2.0), 1e-12);
}

TEST(GridAStar, BlocksLethalAndInscribedCellsButDoesNotWeightInflationCosts)
{
  Costmap2D map(3, 1, 1.0, 0.0, 0.0);
  map.setCost(1, 0, 252);
  EXPECT_DOUBLE_EQ(length(plan(map, 0.5, 0.5, 2.5, 0.5)), 2.0);
  for (const auto cost : {INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE}) {
    map.setCost(1, 0, cost);
    EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 0.5, true), PlannerException);
  }
  map.setCost(1, 0, 0);
  map.setCost(0, 0, LETHAL_OBSTACLE);
  EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 0.5), PlannerException);
  map.setCost(0, 0, 0);
  map.setCost(2, 0, INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 0.5), PlannerException);
}

TEST(GridAStar, SameCellAndIdenticalPositionsPreserveGoalHeading)
{
  Costmap2D map(1, 1, 1.0, 0.0, 0.0);
  EXPECT_EQ(plan(map, 0.1, 0.2, 0.8, 0.9).poses.size(), 2u);
  auto start = pose(0.5, 0.5);
  auto goal = start;
  goal.pose.orientation.z = 2.0;
  goal.pose.orientation.w = 0.0;
  const auto path = createShortestPath(map, "map", builtin_interfaces::msg::Time{}, start, goal);
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.z, 1.0);
  map.setCost(0, 0, LETHAL_OBSTACLE);
  EXPECT_THROW(plan(map, 0.5, 0.5, 0.5, 0.5), PlannerException);
}

TEST(GridAStar, RejectsBoundsFramesInvalidPosesAndEmptyMap)
{
  Costmap2D map(3, 3, 1.0, 0.0, 0.0);
  EXPECT_THROW(plan(map, -0.01, 0.5, 2.5, 2.5), PlannerException);
  EXPECT_THROW(plan(map, 0.5, 0.5, 3.0, 2.5), PlannerException);
  EXPECT_THROW(plan(map, 0.5, -0.1, 2.5, 2.5), PlannerException);
  EXPECT_THROW(plan(map, 0.5, 0.5, 2.5, 3.0), PlannerException);
  EXPECT_THROW(plan(map, 0.5, 0.5, 1e300, 2.5), PlannerException);
  EXPECT_THROW(plan(map, NAN, 0.5, 2.5, 2.5), PlannerException);
  EXPECT_THROW(plan(map, 0.5, 0.5, INFINITY, 2.5), PlannerException);
  auto bad = pose(0.5, 0.5);
  const builtin_interfaces::msg::Time stamp;
  bad.header.frame_id = "odom";
  EXPECT_THROW(createShortestPath(map, "map", stamp, bad, pose(2.5, 2.5)), PlannerException);
  bad.header.frame_id = "map";
  bad.pose.orientation.w = 0.0;
  EXPECT_THROW(createShortestPath(map, "map", stamp, bad, pose(2.5, 2.5)), PlannerException);
  bad.pose.orientation.w = NAN;
  EXPECT_THROW(createShortestPath(map, "map", stamp, pose(2.5, 2.5), bad), PlannerException);
  Costmap2D empty;
  EXPECT_THROW(plan(empty, 0.5, 0.5, 0.5, 0.5), PlannerException);
}

// Independent dense Dijkstra oracle: select the nearest unsettled vertex by a
// linear scan and enumerate candidate edges over all cells. No A* heap/heuristic.
double referenceDistance(const Costmap2D & map)
{
  const int width = map.getSizeInCellsX();
  const int count = width * map.getSizeInCellsY();
  std::vector<double> distances(count, std::numeric_limits<double>::infinity());
  std::vector<bool> settled(count, false);
  distances[0] = 0.0;
  auto free = [&](int x, int y) {return map.getCost(x, y) < INSCRIBED_INFLATED_OBSTACLE;};
  for (int step = 0; step < count; ++step) {
    int u = -1;
    for (int v = 0; v < count; ++v) {
      if (!settled[v] && (u < 0 || distances[v] < distances[u])) {
        u = v;
      }
    }
    if (u < 0 || !std::isfinite(distances[u])) {
      break;
    }
    settled[u] = true;
    for (int v = 0; v < count; ++v) {
      const int dx = std::abs(u % width - v % width);
      const int dy = std::abs(u / width - v / width);
      if (settled[v] || dx > 1 || dy > 1 || !free(v % width, v / width)) {
        continue;
      }
      if (dx == 1 && dy == 1 && (!free(u % width, v / width) || !free(v % width, u / width))) {
        continue;
      }
      distances[v] = std::min(distances[v], distances[u] + std::hypot(dx, dy));
    }
  }
  return distances.back() * map.getResolution();
}

TEST(GridAStar, MatchesIndependentDijkstraOnSeededMapsAndIsDeterministic)
{
  std::mt19937 random(42);
  for (int trial = 0; trial < 100; ++trial) {
    SCOPED_TRACE(trial);
    Costmap2D map(7, 6, 0.5, 0.0, 0.0);
    for (unsigned int y = 0; y < 6; ++y) {
      for (unsigned int x = 0; x < 7; ++x) {
        if (random() % 100 < 28) {
          map.setCost(x, y, LETHAL_OBSTACLE);
        }
      }
    }
    map.setCost(0, 0, 0);
    map.setCost(6, 5, 0);
    const double expected = referenceDistance(map);
    if (std::isfinite(expected)) {
      const auto path = plan(map, 0.25, 0.25, 3.25, 2.75);
      EXPECT_NEAR(length(path), expected, 1e-10);
      expectValidCells(map, path);
      EXPECT_EQ(path, plan(map, 0.25, 0.25, 3.25, 2.75));
    } else {
      EXPECT_THROW(plan(map, 0.25, 0.25, 3.25, 2.75), PlannerException);
    }
  }
}

}  // namespace
