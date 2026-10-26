#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

class PluginLoading : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(PluginLoading, DiscoversInstalledPluginAndRejectsUnconfiguredPlanning)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("mecanumbot_path_planner/AStarPlanner");
  ASSERT_NE(planner, nullptr);
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW(planner->createPlan(pose, pose), nav2_core::PlannerException);
  EXPECT_THROW(planner->activate(), std::logic_error);
  EXPECT_NO_THROW(planner->cleanup());
}

TEST_F(PluginLoading, AllModesPlanAndValidateConfiguration)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("mecanumbot_path_planner/AStarPlanner");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_plugin_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  ASSERT_TRUE(costmap->set_parameter(
    rclcpp::Parameter("plugins", std::vector<std::string>{})).successful);
  ASSERT_EQ(costmap->configure().label(), "inactive");
  auto * map = costmap->getCostmap();
  map->resizeMap(9, 5, 1.0, 0.0, 0.0);
  map->resetMap(0, 0, 9, 5);
  for (unsigned int x = 1; x < 8; ++x) {map->setCost(x, 2, 220);}
  node->declare_parameter("GridBased.optimization_mode", std::string("shortest"));
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 2.5;
  pose.pose.orientation.w = 1.0;
  auto goal = pose;
  goal.pose.position.x = 8.5;

  for (const std::string mode : {"shortest", "safest", "smoothest", "combined"}) {
    ASSERT_TRUE(node->set_parameter(
      rclcpp::Parameter("GridBased.optimization_mode", mode)).successful);
    ASSERT_NO_THROW(planner->configure(node, "GridBased", tf, costmap));
    ASSERT_NO_THROW(planner->activate());
    const auto path = planner->createPlan(pose, goal);
    ASSERT_FALSE(path.poses.empty());
    bool detour = false;
    for (const auto & p : path.poses) {detour |= p.pose.position.y != 2.5;}
    EXPECT_EQ(detour, mode == "safest" || mode == "combined");
    EXPECT_NO_THROW(planner->deactivate());
    EXPECT_THROW(planner->createPlan(pose, pose), nav2_core::PlannerException);
    EXPECT_NO_THROW(planner->cleanup());
  }
  ASSERT_TRUE(node->set_parameter(
    rclcpp::Parameter("GridBased.optimization_mode", "invalid")).successful);
  EXPECT_THROW(planner->configure(node, "GridBased", tf, costmap), std::invalid_argument);
  EXPECT_THROW(planner->activate(), std::logic_error);
  ASSERT_TRUE(node->set_parameter(
    rclcpp::Parameter("GridBased.optimization_mode", "safest")).successful);
  for (const std::string parameter : {
    "GridBased.safety_preference", "GridBased.smoothness_preference"})
  {
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(parameter, -1.0)).successful);
    EXPECT_THROW(planner->configure(node, "GridBased", tf, costmap), std::invalid_argument);
    EXPECT_THROW(planner->activate(), std::logic_error);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(parameter, 0.0)).successful);
  }
  planner->configure(node, "GridBased", tf, costmap);
  planner->activate();
  for (const auto & p : planner->createPlan(pose, goal).poses) {
    EXPECT_DOUBLE_EQ(p.pose.position.y, 2.5);  // Zero safety preference restores shortest.
  }
  planner->deactivate();
  planner->cleanup();
  EXPECT_EQ(costmap->cleanup().label(), "unconfigured");
}

TEST_F(PluginLoading, ShortestUsesCostmapAndRespectsUnknownParameterWithoutModifyingMap)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("mecanumbot_path_planner/AStarPlanner");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("shortest_plugin_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("shortest_costmap");
  planner->configure(node, "GridBased", tf, costmap);
  planner->activate();
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position.x = 0.5;
  start.pose.position.y = 0.5;
  start.pose.orientation.w = 1.0;
  auto goal = start;
  goal.pose.position.x = 2.5;
  EXPECT_THROW(planner->createPlan(start, goal), nav2_core::PlannerException);

  // Configure an empty costmap with no sensor layers. Do not activate it or a robot.
  ASSERT_TRUE(costmap->set_parameter(
    rclcpp::Parameter("plugins", std::vector<std::string>{})).successful);
  ASSERT_EQ(costmap->configure().label(), "inactive");
  auto * map = costmap->getCostmap();
  map->resizeMap(3, 1, 1.0, 0.0, 0.0);
  map->resetMap(0, 0, 3, 1);
  const auto path = planner->createPlan(start, goal);
  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_GT(path.header.stamp.sec, 0);
  map->setCost(1, 0, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_THROW(planner->createPlan(start, goal), nav2_core::PlannerException);
  planner->deactivate();
  EXPECT_THROW(planner->createPlan(start, goal), nav2_core::PlannerException);
  planner->cleanup();
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("GridBased.allow_unknown", true)).successful);
  planner->configure(node, "GridBased", tf, costmap);
  planner->activate();
  EXPECT_EQ(planner->createPlan(start, goal).poses.size(), 3u);
  EXPECT_EQ(map->getCost(1, 0), nav2_costmap_2d::NO_INFORMATION);
  planner->deactivate();
  planner->cleanup();
  EXPECT_EQ(costmap->cleanup().label(), "unconfigured");
}

}  // namespace
