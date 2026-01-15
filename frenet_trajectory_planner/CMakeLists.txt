cmake_minimum_required(VERSION 3.8)
project(frenet_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 REQUIRED)
endif()

# if(NOT "${CMAKE_CXX_STANDARD}")
#     set(CMAKE_CXX_STANDARD 17)
#   endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)

add_library(frenet_trajectory_planner_lib SHARED
  src/frenet_trajectory_generator.cpp
  src/quartic_trajectory_planner.cpp
  src/quintic_trajectory_planner.cpp)

target_include_directories(
  frenet_trajectory_planner_lib
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${angles_INCLUDE_DIRS}>)

target_include_directories(frenet_trajectory_planner_lib PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(frenet_trajectory_planner_lib PUBLIC Eigen3::Eigen)
target_link_libraries(frenet_trajectory_planner_lib PRIVATE
  angles::angles
)

install(
  TARGETS frenet_trajectory_planner_lib
  EXPORT frenet_trajectory_planner_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/frenet_trajectory_planner")
ament_export_dependencies(Eigen3)
ament_export_targets(frenet_trajectory_planner_lib)

add_subdirectory(demos)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
