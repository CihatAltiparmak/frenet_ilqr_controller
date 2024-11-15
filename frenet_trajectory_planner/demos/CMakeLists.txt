add_executable(
  generate_all_possible_trajectories_demo
  generate_all_possible_trajectories_demo.cpp)

target_link_libraries(generate_all_possible_trajectories_demo frenet_trajectory_planner_lib)

add_executable(
  select_best_trajectory_demo
  select_best_trajectory_demo.cpp)

target_link_libraries(select_best_trajectory_demo frenet_trajectory_planner_lib)

add_executable(
  plan_trajectory_demo
  plan_trajectory_demo.cpp
  # ../src/frenet_trajectory_planner.cpp
  )

target_link_libraries(plan_trajectory_demo frenet_trajectory_planner_lib)

add_executable(
  plan_through_waypoint_demo
  plan_through_waypoint_demo.cpp
  # ../src/frenet_trajectory_planner.cpp
  )

target_link_libraries(plan_through_waypoint_demo frenet_trajectory_planner_lib)

add_executable(
  circle_adapter_demo
  adapters/circle_adapter_demo.cpp
  )

target_link_libraries(circle_adapter_demo frenet_trajectory_planner_lib)

install(TARGETS generate_all_possible_trajectories_demo
        select_best_trajectory_demo
        plan_trajectory_demo
        plan_through_waypoint_demo
        circle_adapter_demo
        DESTINATION lib/frenet_trajectory_planner)