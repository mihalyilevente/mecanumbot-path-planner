#ifndef MECANUMBOT_PATH_PLANNER__A_STAR_PLANNER_HPP_
#define MECANUMBOT_PATH_PLANNER__A_STAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"

namespace mecanumbot_path_planner
{

// Humble GlobalPlanner scaffold. Search and criterion costs are not implemented yet.
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::string optimization_mode_{"shortest"};
  bool configured_{false};
  bool active_{false};
};

}  // namespace mecanumbot_path_planner

#endif  // MECANUMBOT_PATH_PLANNER__A_STAR_PLANNER_HPP_
