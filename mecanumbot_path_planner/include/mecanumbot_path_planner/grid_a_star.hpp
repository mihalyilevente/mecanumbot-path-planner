#ifndef MECANUMBOT_PATH_PLANNER__GRID_A_STAR_HPP_
#define MECANUMBOT_PATH_PLANNER__GRID_A_STAR_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mecanumbot_path_planner/planner_options.hpp"

namespace mecanumbot_path_planner
{

// Safest: length + safety_weight * integral of normalized costmap cost.
// Smoothest: length + smoothness_weight * sum of squared travel-direction changes.
// Combined: length + weighted cost exposure + weighted direction changes.
// See README for discrete edge costs, endpoint connectors and scope of guarantees.
nav_msgs::msg::Path createGridPath(
  const nav2_costmap_2d::Costmap2D & map, const std::string & global_frame,
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const PlannerOptions & options);

// The caller must provide an immutable snapshot (or hold the costmap mutex).
// Optimizes geometric length on the 8-connected cell-center graph. Exact endpoint
// connectors remain within their cells. Throws nav2_core::PlannerException on failure.
nav_msgs::msg::Path createShortestPath(
  const nav2_costmap_2d::Costmap2D & map, const std::string & global_frame,
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, bool allow_unknown = false);

}  // namespace mecanumbot_path_planner

#endif  // MECANUMBOT_PATH_PLANNER__GRID_A_STAR_HPP_
