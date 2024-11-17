find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

find_package(ament_cmake_gtest REQUIRED)

# Disable this test until revized with new form of acceleration_policy
# ament_add_gtest(
#   acceleration_policy_test
#   acceleration_policy_test.cpp)
# 
# target_link_libraries(acceleration_policy_test  frenet_trajectory_planner_lib)

ament_add_gtest(
  quintic_trajectory_planner_test
  quintic_trajectory_planner_test.cpp)

target_link_libraries(quintic_trajectory_planner_test  frenet_trajectory_planner_lib)

ament_add_gtest(
  quartic_trajectory_planner_test
  quartic_trajectory_planner_test.cpp)

target_link_libraries(quartic_trajectory_planner_test frenet_trajectory_planner_lib)

ament_add_gtest(
  frenet_trajectory_generator_test
  frenet_trajectory_generator_test.cpp)

target_link_libraries(frenet_trajectory_generator_test  frenet_trajectory_planner_lib)

ament_add_gtest(
  line_adapter_test
  conversion_adapters/line_adapter_test.cpp)

target_link_libraries(line_adapter_test frenet_trajectory_planner_lib)

ament_add_gtest(
  circle_adapter_test
  conversion_adapters/circle_adapter_test.cpp)

target_link_libraries(circle_adapter_test frenet_trajectory_planner_lib)

ament_add_gtest(
  frenet_frame_converter_test
  frenet_frame_converter_test.cpp)

target_link_libraries(frenet_frame_converter_test frenet_trajectory_planner_lib)

# ament_add_gtest(
#   frenet_trajectory_selector_test
#   frenet_trajectory_selector_test.cpp)
# 
# target_link_libraries(frenet_trajectory_selector_test  frenet_trajectory_planner_lib)

# if(TARGET Eigen3::Eigen)
#   target_link_libraries(quintic_trajectory_planner_test PUBLIC Eigen3::Eigen)
# else()
#   target_include_directories(quintic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
# endif()