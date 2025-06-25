cmake_minimum_required(VERSION 3.5)
project(nav2_frenet_ilqr_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(frenet_trajectory_planner REQUIRED)
find_package(ilqr_trajectory_tracker REQUIRED)
find_package(nav2_ros_common REQUIRED)

include_directories(
  include
)

nav2_package()

set(library_name nav2_frenet_ilqr_controller)

add_library(${library_name} SHARED
  src/frenet_ilqr_controller.cpp
  src/path_handler.cpp
  src/parameter_handler.cpp)
<<<<<<< HEAD

ament_target_dependencies(${library_name}
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  angles
  frenet_trajectory_planner
  ilqr_trajectory_tracker
  rcl_interfaces
=======
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  frenet_trajectory_planner::frenet_trajectory_planner_lib
  ilqr_trajectory_tracker::ilqr_trajectory_tracker_lib
)
target_link_libraries(${library_name} PRIVATE
  angles::angles
>>>>>>> 443563f (Migrate nav2_utils's ros relavant dependencies to nav2_ros_common (#44))
)

add_library(nav2_frenet_ilqr_controller_costs SHARED
  src/costs/lateral_distance_cost.cpp
  src/costs/longtitutal_velocity_cost.cpp
  src/costs/obstacle_cost.cpp)

<<<<<<< HEAD
ament_target_dependencies(nav2_frenet_ilqr_controller_costs
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  angles
  frenet_trajectory_planner
  ilqr_trajectory_tracker
  rcl_interfaces
=======
target_include_directories(nav2_frenet_ilqr_controller_costs
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)

target_link_libraries(nav2_frenet_ilqr_controller_costs PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  frenet_trajectory_planner::frenet_trajectory_planner_lib
)

target_link_libraries(nav2_frenet_ilqr_controller_costs PRIVATE
  pluginlib::pluginlib
>>>>>>> 443563f (Migrate nav2_utils's ros relavant dependencies to nav2_ros_common (#44))
)

add_library(nav2_frenet_ilqr_controller_policies SHARED
  src/policies/obstacle_policy.cpp)

<<<<<<< HEAD
ament_target_dependencies(nav2_frenet_ilqr_controller_policies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  angles
  frenet_trajectory_planner
  ilqr_trajectory_tracker
  rcl_interfaces
=======
target_include_directories(nav2_frenet_ilqr_controller_policies
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)

target_link_libraries(nav2_frenet_ilqr_controller_policies PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  frenet_trajectory_planner::frenet_trajectory_planner_lib
)

target_link_libraries(nav2_frenet_ilqr_controller_policies PRIVATE
  pluginlib::pluginlib
>>>>>>> 443563f (Migrate nav2_utils's ros relavant dependencies to nav2_ros_common (#44))
)

install(TARGETS ${library_name} nav2_frenet_ilqr_controller_costs nav2_frenet_ilqr_controller_policies
  EXPORT ${library_name} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# 
#   find_package(tf2_geometry_msgs REQUIRED)
# 
#   ament_find_gtest()
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_msgs
  tf2
  tf2_ros
  nav2_ros_common
)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_frenet_ilqr_controller.xml)
pluginlib_export_plugin_description_file(nav2_frenet_ilqr_controller policies.xml)
pluginlib_export_plugin_description_file(nav2_frenet_ilqr_controller costs.xml)

ament_package()