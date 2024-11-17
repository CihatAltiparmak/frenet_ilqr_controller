#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_planner.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <iostream>

int main()
{
  using frenet_trajectory_planner::CartesianPoint;

  frenet_trajectory_planner::CartesianState robot_cartesian_state =
    frenet_trajectory_planner::CartesianState::Zero();
  robot_cartesian_state[3] = -0.5;
  robot_cartesian_state[0] = 1;
  robot_cartesian_state[1] = 0.0011;

  std::vector<CartesianPoint> waypoint_list;
  {
    CartesianPoint waypoint;
    waypoint << 1, -1;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    waypoint << 5, -1;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    waypoint << 6, -2;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    waypoint << 6, -3;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 5, -3; // 7, -4;
    waypoint << 7, -4;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 3, -5; // 9, -5;
    waypoint << 9, -5;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 3, -5; // 9, -5;
    waypoint << 5, -6;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 3, -5; // 9, -5;
    waypoint << 5, -8;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 3, -5; // 9, -5;
    waypoint << 3, -3;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    // waypoint << 3, -5; // 9, -5;
    waypoint << -2, -3;
    waypoint_list.push_back(waypoint);
  }

  {
    CartesianPoint waypoint;
    waypoint << 1, -1;
    waypoint_list.push_back(waypoint);
  }

  auto frenet_trajectory_planner = frenet_trajectory_planner::FrenetTrajectoryPlanner();
  auto planned_cartesian_trajectory = frenet_trajectory_planner.plan_by_waypoint(
    robot_cartesian_state,
    waypoint_list, 1.0);

  for (const auto & cartesian_state : planned_cartesian_trajectory) {
    std::cout << cartesian_state[0] << ", " << cartesian_state[3] << std::endl;
  }
  std::cout << "END" << std::endl;

  for (const auto & cartesian_state : planned_cartesian_trajectory) {
    if (cartesian_state({2, 5}).norm() >= 10) {
      std::cout << cartesian_state[0] << ", " << cartesian_state[3] << std::endl;
      std::cerr << "LETS FUCKIN GO : " << cartesian_state[0] << ", " << cartesian_state[3] <<
        std::endl;
    }
  }
  std::cout << "END" << std::endl;

  for (auto waypoint : waypoint_list) {
    std::cout << waypoint[0] << ", " << waypoint[1] << std::endl;
  }
  std::cout << "END" << std::endl;

  // for (size_t i = 0; i <  planned_cartesian_trajectory.size() - 2; i++) {
  //   auto cs1 = planned_cartesian_trajectory[i]({0, 3});
  //   auto cs2 = planned_cartesian_trajectory[i + 1]({0, 3});
  //   auto cs3 = planned_cartesian_trajectory[i + 2]({0, 3});

  //   double vel1 = (cs2 - cs1).norm() / 0.01;
  //   double vel2 = (cs3 - cs2).norm() / 0.01;

  //   double accel = (vel2 - vel1) / 0.01;

  //   if (accel > 1 || accel < -1) {
  //     std::cerr << i << " | " << accel << std::endl;
  //     std::cout << cs1[0] << ", " << cs1[1] << std::endl;
  //     std::cout << cs2[0] << ", " << cs2[1] << std::endl;
  //     std::cout << cs3[0] << ", " << cs3[1] << std::endl;
  //   }
  // }
  // std::cout << "END" << std::endl;
  return 0;
}
