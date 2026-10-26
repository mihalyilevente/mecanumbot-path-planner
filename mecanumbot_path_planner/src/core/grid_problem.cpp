#include "grid_problem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace mecanumbot_path_planner::detail
{
namespace
{

geometry_msgs::msg::PoseStamped preparePose(
  const geometry_msgs::msg::PoseStamped & input,
  const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  if (input.header.frame_id != frame) {
    throw nav2_core::PlannerException("Start and goal must be in the costmap global frame");
  }

  auto result = input;
  auto & position = result.pose.position;
  auto & orientation = result.pose.orientation;
  const double quaternion_norm = std::hypot(
    std::hypot(orientation.x, orientation.y),
    std::hypot(orientation.z, orientation.w));

  const bool valid =
    std::isfinite(position.x) && std::isfinite(position.y) &&
    std::isfinite(position.z) && std::isfinite(quaternion_norm) &&
    quaternion_norm >= 1e-12;
  if (!valid) {
    throw nav2_core::PlannerException(
            "Poses require finite coordinates and a nonzero quaternion");
  }

  orientation.x /= quaternion_norm;
  orientation.y /= quaternion_norm;
  orientation.z /= quaternion_norm;
  orientation.w /= quaternion_norm;
  result.header.stamp = stamp;
  return result;
}

void validateMap(const nav2_costmap_2d::Costmap2D & map, const std::string & frame)
{
  const double end_x = map.getOriginX() + map.getSizeInCellsX() * map.getResolution();
  const double end_y = map.getOriginY() + map.getSizeInCellsY() * map.getResolution();
  const bool valid =
    map.getSizeInCellsX() > 0 && map.getSizeInCellsY() > 0 &&
    std::isfinite(map.getResolution()) && map.getResolution() > 0.0 &&
    std::isfinite(map.getOriginX()) && std::isfinite(map.getOriginY()) &&
    std::isfinite(end_x) && std::isfinite(end_y) && !frame.empty();
  if (!valid) {
    throw nav2_core::PlannerException("Costmap geometry and global frame must be valid");
  }
}

}  // namespace

const std::array<int, GridProblem::kDirectionCount> GridProblem::kDirectionX{
  {-1, 0, 1, -1, 1, -1, 0, 1}};
const std::array<int, GridProblem::kDirectionCount> GridProblem::kDirectionY{
  {-1, -1, -1, 0, 0, 1, 1, 1}};

GridProblem::GridProblem(
  const nav2_costmap_2d::Costmap2D & input_map,
  const std::string & global_frame,
  const builtin_interfaces::msg::Time & planning_stamp,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerOptions & planner_options)
: map(input_map),
  frame(global_frame),
  stamp(planning_stamp),
  options(planner_options),
  width(input_map.getSizeInCellsX()),
  height(input_map.getSizeInCellsY()),
  resolution(input_map.getResolution()),
  start_pose(preparePose(start, global_frame, planning_stamp)),
  goal_pose(preparePose(goal, global_frame, planning_stamp)),
  start_cell(worldToCell(start_pose)),
  goal_cell(worldToCell(goal_pose)),
  has_start_connector([&]() {
      double x, y;
      map.mapToWorld(start_cell % width, start_cell / width, x, y);
      return x != start_pose.pose.position.x || y != start_pose.pose.position.y;
    }()),
  has_goal_connector([&]() {
      double x, y;
      map.mapToWorld(goal_cell % width, goal_cell / width, x, y);
      return x != goal_pose.pose.position.x || y != goal_pose.pose.position.y;
    }()),
  start_connector_angle([&]() {
      double x, y;
      map.mapToWorld(start_cell % width, start_cell / width, x, y);
      return std::atan2(y - start_pose.pose.position.y, x - start_pose.pose.position.x);
    }()),
  goal_connector_angle([&]() {
      double x, y;
      map.mapToWorld(goal_cell % width, goal_cell / width, x, y);
      return std::atan2(goal_pose.pose.position.y - y, goal_pose.pose.position.x - x);
    }())
{
  validatePlannerOptions(options);
  validateMap(map, frame);
  if (!traversable(start_cell) || !traversable(goal_cell)) {
    throw nav2_core::PlannerException(
            "Start or goal cell is occupied or disallowed unknown space");
  }
}

std::size_t GridProblem::worldToCell(const geometry_msgs::msg::PoseStamped & pose) const
{
  validateMap(map, frame);
  const auto & position = pose.pose.position;
  const double end_x = map.getOriginX() + width * resolution;
  const double end_y = map.getOriginY() + height * resolution;
  unsigned int x, y;
  const bool inside =
    position.x >= map.getOriginX() && position.y >= map.getOriginY() &&
    position.x < end_x && position.y < end_y &&
    map.worldToMap(position.x, position.y, x, y);
  if (!inside) {
    throw nav2_core::PlannerException("Start or goal is outside the costmap");
  }
  return static_cast<std::size_t>(y) * width + x;
}

bool GridProblem::traversable(std::size_t cell) const
{
  const auto cost = map.getCost(cell % width, cell / width);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return options.allow_unknown;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

std::vector<Neighbor> GridProblem::neighbors(std::size_t cell) const
{
  std::vector<Neighbor> result;
  const auto x = static_cast<std::int64_t>(cell % width);
  const auto y = static_cast<std::int64_t>(cell / width);

  for (std::size_t direction = 0; direction < kDirectionCount; ++direction) {
    const int dx = kDirectionX[direction];
    const int dy = kDirectionY[direction];
    const auto next_x = x + dx;
    const auto next_y = y + dy;
    if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) {
      continue;
    }

    const auto next = static_cast<std::size_t>(next_y) * width + next_x;
    if (!traversable(next)) {
      continue;
    }

    const bool diagonal = dx != 0 && dy != 0;
    if (diagonal &&
      (!traversable(static_cast<std::size_t>(y) * width + next_x) ||
      !traversable(static_cast<std::size_t>(next_y) * width + x)))
    {
      continue;
    }

    result.push_back({next, direction, resolution * (diagonal ? std::sqrt(2.0) : 1.0)});
  }
  return result;
}

double GridProblem::heuristic(std::size_t cell) const
{
  const double dx = std::abs(
    static_cast<double>(cell % width) - static_cast<double>(goal_cell % width));
  const double dy = std::abs(
    static_cast<double>(cell / width) - static_cast<double>(goal_cell / width));
  return resolution * (std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy));
}

double GridProblem::risk(std::size_t cell) const
{
  const auto cost = map.getCost(cell % width, cell / width);
  return cost == nav2_costmap_2d::NO_INFORMATION ? 1.0 : cost / 252.0;
}

double GridProblem::directionAngle(std::size_t direction) const
{
  return std::atan2(kDirectionY[direction], kDirectionX[direction]);
}

double GridProblem::turnCost(double from, double to) const
{
  const double angle = std::atan2(std::sin(to - from), std::cos(to - from));
  return options.smoothnessWeight() * angle * angle;
}

bool samePosition(
  const geometry_msgs::msg::PoseStamped & left,
  const geometry_msgs::msg::PoseStamped & right)
{
  return left.pose.position.x == right.pose.position.x &&
         left.pose.position.y == right.pose.position.y &&
         left.pose.position.z == right.pose.position.z;
}

}  // namespace mecanumbot_path_planner::detail
