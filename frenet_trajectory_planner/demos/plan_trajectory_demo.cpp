#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_planner.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <iostream>

int main()
{

  frenet_trajectory_planner::CartesianState robot_cartesian_state =
    frenet_trajectory_planner::CartesianState::Zero();
  robot_cartesian_state[1] = 1;
  frenet_trajectory_planner::CartesianPoint start_point;
  start_point << 1, 1;
  frenet_trajectory_planner::CartesianPoint final_point;
  final_point << 5, 1;


  auto frenet_trajectory_planner = frenet_trajectory_planner::FrenetTrajectoryPlanner();
  auto planned_cartesian_trajectory = frenet_trajectory_planner.plan(
    robot_cartesian_state,
    start_point, final_point);

  for (const auto & cartesian_state : planned_cartesian_trajectory) {
    std::cout << cartesian_state[0] << ", " << cartesian_state[3] << std::endl;
  }
  std::cout << "END" << std::endl;
  return 0;
}
