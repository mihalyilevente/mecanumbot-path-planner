cmake_minimum_required(VERSION 3.8)
project(mecanumbot_path_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(dependencies
  builtin_interfaces geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_util
  pluginlib rclcpp rclcpp_lifecycle tf2_ros
)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/core/grid_a_star.cpp
  src/core/grid_problem.cpp
  src/core/path_builder.cpp
  src/core/planner_options.cpp
  src/planners/combined_path.cpp
  src/planners/safest_path.cpp
  src/planners/shortest_path.cpp
  src/planners/smoothest_path.cpp
  src/plugin/a_star_planner.cpp
  src/search/cell_search.cpp
  src/search/directional_search.cpp
  src/search/search_queue.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Installs plugin.xml and registers it in the nav2_core ament resource index.
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_loading test/integration/test_plugin_loading.cpp)
  # Deliberately do not link our library: pluginlib must discover and load it.
  ament_target_dependencies(test_plugin_loading ${dependencies})
  ament_add_gtest(test_grid_a_star test/unit/test_grid_a_star.cpp)
  target_link_libraries(test_grid_a_star ${PROJECT_NAME})
  ament_target_dependencies(test_grid_a_star ${dependencies})
  ament_add_gtest(test_criteria test/unit/test_criteria.cpp)
  target_link_libraries(test_criteria ${PROJECT_NAME})
  ament_target_dependencies(test_criteria ${dependencies})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
