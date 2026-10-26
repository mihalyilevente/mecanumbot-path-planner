#include "mecanumbot_path_planner/a_star_planner.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "mecanumbot_path_planner/grid_a_star.hpp"

namespace mecanumbot_path_planner
{
namespace
{

PlannerOptions readOptions(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & planner_name)
{
  PlannerOptions options;
  const auto key = [&planner_name](const std::string & suffix) {
      return planner_name + "." + suffix;
    };

  nav2_util::declare_parameter_if_not_declared(
    node, key("optimization_mode"), rclcpp::ParameterValue(std::string("shortest")));
  nav2_util::declare_parameter_if_not_declared(
    node, key("allow_unknown"), rclcpp::ParameterValue(options.allow_unknown));
  nav2_util::declare_parameter_if_not_declared(
    node, key("safety_preference"), rclcpp::ParameterValue(options.safety_preference));
  nav2_util::declare_parameter_if_not_declared(
    node, key("smoothness_preference"), rclcpp::ParameterValue(options.smoothness_preference));

  options.mode = parseOptimizationMode(
    node->get_parameter(key("optimization_mode")).as_string());
  options.allow_unknown = node->get_parameter(key("allow_unknown")).as_bool();
  options.safety_preference = node->get_parameter(key("safety_preference")).as_double();
  options.smoothness_preference =
    node->get_parameter(key("smoothness_preference")).as_double();
  validatePlannerOptions(options);
  return options;
}

nav2_costmap_2d::Costmap2D snapshotCostmap(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  if (!costmap_ros || !costmap_ros->getLayeredCostmap()) {
    throw nav2_core::PlannerException("AStarPlanner: costmap is not initialized");
  }
  auto * live_map = costmap_ros->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live_map->getMutex());
  return *live_map;
}

}  // namespace

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

  const auto options = readOptions(node, name);
  const auto mode = node->get_parameter(name + ".optimization_mode").as_string();

  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  options_ = options;
  configured_ = true;
  active_ = false;
  RCLCPP_INFO(
    node->get_logger(), "%s configured in %s mode",
    name_.c_str(), mode.c_str());
}

void AStarPlanner::cleanup()
{
  active_ = false;
  configured_ = false;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
  name_.clear();
  options_ = PlannerOptions{};
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
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto node = node_.lock();
  if (!configured_ || !active_ || !node) {
    throw nav2_core::PlannerException("AStarPlanner is not configured and active");
  }
  const auto snapshot = snapshotCostmap(costmap_ros_);
  return createGridPath(
    snapshot, costmap_ros_->getGlobalFrameID(), node->now(),
    start, goal, options_);
}

}  // namespace mecanumbot_path_planner

PLUGINLIB_EXPORT_CLASS(mecanumbot_path_planner::AStarPlanner, nav2_core::GlobalPlanner)
