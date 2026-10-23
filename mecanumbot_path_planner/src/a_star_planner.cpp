#include "mecanumbot_path_planner/a_star_planner.hpp"

#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanumbot_path_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (configured_) {
    throw std::logic_error("AStarPlanner must be cleaned up before reconfiguration");
  }
  auto node = parent.lock();
  if (!node || !tf || !costmap_ros || name.empty()) {
    throw std::invalid_argument("AStarPlanner requires a parent, name, TF buffer and costmap");
  }

  const auto parameter = name + ".optimization_mode";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter, rclcpp::ParameterValue(std::string("shortest")));
  const auto mode = node->get_parameter(parameter).as_string();
  if (mode != "shortest" && mode != "safest" && mode != "smoothest") {
    throw std::invalid_argument(parameter + " must be shortest, safest or smoothest");
  }

  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  optimization_mode_ = mode;
  configured_ = true;
  active_ = false;
  RCLCPP_INFO(
    node->get_logger(), "%s configured in %s mode (planner skeleton only)",
    name_.c_str(), optimization_mode_.c_str());
}

void AStarPlanner::cleanup()
{
  active_ = false;
  configured_ = false;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
  name_.clear();
  optimization_mode_ = "shortest";
}

void AStarPlanner::activate()
{
  if (!configured_ || node_.expired()) {
    throw std::logic_error("AStarPlanner must be configured before activation");
  }
  active_ = true;
}

void AStarPlanner::deactivate()
{
  active_ = false;
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & /*start*/,
  const geometry_msgs::msg::PoseStamped & /*goal*/)
{
  if (!configured_ || !active_ || node_.expired()) {
    throw nav2_core::PlannerException("AStarPlanner is not configured and active");
  }
  // TODO: Add costmap-based A* and criterion costs in a separate development task.
  // A fabricated straight-line path would not establish collision-free planning.
  throw nav2_core::PlannerException(
          "AStarPlanner: path search is not implemented (" + optimization_mode_ + " mode)");
}

}  // namespace mecanumbot_path_planner

PLUGINLIB_EXPORT_CLASS(mecanumbot_path_planner::AStarPlanner, nav2_core::GlobalPlanner)
