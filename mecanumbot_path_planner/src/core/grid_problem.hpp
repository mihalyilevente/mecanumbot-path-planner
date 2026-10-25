#ifndef MECANUMBOT_PATH_PLANNER__GRID_PROBLEM_HPP_
#define MECANUMBOT_PATH_PLANNER__GRID_PROBLEM_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mecanumbot_path_planner/planner_options.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mecanumbot_path_planner::detail
{

struct Neighbor
{
  std::size_t cell;
  std::size_t direction;
  double distance;
};

class GridProblem
{
public:
  GridProblem(
    const nav2_costmap_2d::Costmap2D & map, const std::string & global_frame,
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerOptions & options);

  std::vector<Neighbor> neighbors(std::size_t cell) const;
  bool traversable(std::size_t cell) const;
  double heuristic(std::size_t cell) const;
  double risk(std::size_t cell) const;
  double directionAngle(std::size_t direction) const;
  double turnCost(double from, double to) const;

  const nav2_costmap_2d::Costmap2D & map;
  const std::string frame;
  const builtin_interfaces::msg::Time stamp;
  const PlannerOptions options;
  const unsigned int width;
  const unsigned int height;
  const double resolution;
  const geometry_msgs::msg::PoseStamped start_pose;
  const geometry_msgs::msg::PoseStamped goal_pose;
  const std::size_t start_cell;
  const std::size_t goal_cell;
  const bool has_start_connector;
  const bool has_goal_connector;
  const double start_connector_angle;
  const double goal_connector_angle;

  static constexpr std::size_t kDirectionCount = 8;
  static const std::array<int, kDirectionCount> kDirectionX;
  static const std::array<int, kDirectionCount> kDirectionY;

private:
  std::size_t worldToCell(const geometry_msgs::msg::PoseStamped & pose) const;
};

bool samePosition(
  const geometry_msgs::msg::PoseStamped & left,
  const geometry_msgs::msg::PoseStamped & right);

nav_msgs::msg::Path buildPath(
  const GridProblem & problem, const std::vector<std::size_t> & cells);

}  // namespace mecanumbot_path_planner::detail

#endif  // MECANUMBOT_PATH_PLANNER__GRID_PROBLEM_HPP_
