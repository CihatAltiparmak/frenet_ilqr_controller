cmake_minimum_required(VERSION 3.8)
project(ilqr_trajectory_tracker)

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

add_library(ilqr_trajectory_tracker_lib SHARED src/models/diff_robot_model.cpp)

target_include_directories(
  ilqr_trajectory_tracker_lib
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(ilqr_trajectory_tracker_lib PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(ilqr_trajectory_tracker_lib  Eigen3::Eigen)

add_subdirectory(demos)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(
  TARGETS ilqr_trajectory_tracker_lib
  EXPORT ilqr_trajectory_tracker_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/ilqr_trajectory_tracker")
ament_export_dependencies(Eigen3)
ament_export_targets(ilqr_trajectory_tracker_lib)

ament_package()

