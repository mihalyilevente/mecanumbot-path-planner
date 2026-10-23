#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
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

TEST_F(PluginLoading, LifecycleAndAllModePlaceholdersReportUnimplementedSearch)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("mecanumbot_path_planner/AStarPlanner");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_plugin_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // No costmap lifecycle, sensors, executor or robot is started in this smoke test.
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  node->declare_parameter("GridBased.optimization_mode", std::string("shortest"));
  geometry_msgs::msg::PoseStamped pose;

  for (const std::string mode : {"shortest", "safest", "smoothest"}) {
    ASSERT_TRUE(node->set_parameter(
      rclcpp::Parameter("GridBased.optimization_mode", mode)).successful);
    ASSERT_NO_THROW(planner->configure(node, "GridBased", tf, costmap));
    ASSERT_NO_THROW(planner->activate());
    try {
      planner->createPlan(pose, pose);
      FAIL() << "The skeleton must not return a path";
    } catch (const nav2_core::PlannerException & error) {
      EXPECT_NE(std::string(error.what()).find("not implemented"), std::string::npos);
      EXPECT_NE(std::string(error.what()).find(mode), std::string::npos);
    }
    EXPECT_NO_THROW(planner->deactivate());
    EXPECT_THROW(planner->createPlan(pose, pose), nav2_core::PlannerException);
    EXPECT_NO_THROW(planner->cleanup());
  }
  ASSERT_TRUE(node->set_parameter(
    rclcpp::Parameter("GridBased.optimization_mode", "invalid")).successful);
  EXPECT_THROW(planner->configure(node, "GridBased", tf, costmap), std::invalid_argument);
  EXPECT_THROW(planner->activate(), std::logic_error);
}

}  // namespace
