#include "grid_problem.hpp"

#include <cmath>

namespace mecanumbot_path_planner::detail
{
namespace
{
geometry_msgs::msg::PoseStamped cellCenter(const GridProblem & problem, std::size_t cell)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = problem.frame;
  pose.header.stamp = problem.stamp;
  problem.map.mapToWorld(
    cell % problem.width, cell / problem.width,
    pose.pose.position.x, pose.pose.position.y);
  pose.pose.orientation.w = 1.0;
  return pose;
}

void pointIntermediatePosesForward(nav_msgs::msg::Path & path)
{
  for (std::size_t index = 1; index + 1 < path.poses.size(); ++index) {
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    path.poses[index].pose.orientation.z = std::sin(yaw / 2.0);
    path.poses[index].pose.orientation.w = std::cos(yaw / 2.0);
  }
}
}  // namespace

nav_msgs::msg::Path buildPath(
  const GridProblem & problem, const std::vector<std::size_t> & cells)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = problem.frame;
  path.header.stamp = problem.stamp;
  path.poses.push_back(problem.start_pose);
  for (const auto cell : cells) {
    const auto center = cellCenter(problem, cell);
    if (!samePosition(path.poses.back(), center)) {
      path.poses.push_back(center);
    }
  }
  if (samePosition(path.poses.back(), problem.goal_pose)) {
    path.poses.back() = problem.goal_pose;
  } else {
    path.poses.push_back(problem.goal_pose);
  }
  pointIntermediatePosesForward(path);
  return path;
}
}  // namespace mecanumbot_path_planner::detail
