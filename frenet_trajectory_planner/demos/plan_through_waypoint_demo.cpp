#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_planner.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <iostream>

int main()
{
  using frenet_trajectory_planner::CartesianPoint;

  frenet_trajectory_planner::CartesianState robot_cartesian_state =
    frenet_trajectory_planner::CartesianState::Zero();
  robot_cartesian_state[0] = -0.323718;
  robot_cartesian_state[1] = -1;
  robot_cartesian_state[3] = 0.002;
  robot_cartesian_state[4] = 0;

  std::cout << robot_cartesian_state[0] << ", " << robot_cartesian_state[3] << std::endl;
  std::cout << "END" << std::endl;

  std::vector<CartesianPoint> waypoint_list;
  {
    CartesianPoint waypoint;
    waypoint << 0.009462, 0.00264300;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << 0.007240, -0.0283170;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.015914, -0.037745;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.038550, -0.048357;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.061019, -0.059317;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.083550, -0.070151;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.107279, -0.078020;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.131147, -0.085457;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.155242, -0.092124;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.179793, -0.096844;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.204482, -0.100769;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.229308, -0.103722;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.254265, -0.105170;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.279261, -0.105620;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.304243, -0.104660;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.329166, -0.102699;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.353964, -0.099529;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.378607, -0.095318;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.403033, -0.089991;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.427210, -0.083631;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.451072, -0.076173;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.474676, -0.067937;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.498012, -0.058966;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.520988, -0.049114;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.543727, -0.038724;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.566129, -0.027628;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.588354, -0.016179;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.610284, -0.004175;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.632108, 0.0080180;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.653714, 0.0205950;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.675281, 0.0332380;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.696792, 0.0459780;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.718280, 0.0587540;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.739750, 0.0715640;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.761199, 0.0844070;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.782456, 0.0975640;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.803788, 0.1106010;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.825355, 0.1232440;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.846978, 0.1357930;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.868749, 0.1480810;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.890569, 0.1602840;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.912553, 0.1721880;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.934548, 0.1840710;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.956558, 0.1959280;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -0.978574, 0.2077710;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.000585, 0.2196260;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.022610, 0.2314530;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.044624, 0.2433020;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.066661, 0.2551060;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.088675, 0.2669550;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.110743, 0.2787020;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.132884, 0.2903130;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.155095, 0.3017880;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.177418, 0.3130420;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.199814, 0.3241510;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.222695, 0.3342250;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.245355, 0.3447860;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.267464, 0.3564550;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.289244, 0.3687290;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.310325, 0.3821660;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.331073, 0.3961150;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.350187, 0.4122280;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.369683, 0.4278780;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.389741, 0.4427990;
    waypoint_list.push_back(waypoint);
  }
  {
    CartesianPoint waypoint;
    waypoint << -1.409953, 0.4575120;
    waypoint_list.push_back(waypoint);
  }

  size_t closest_index = 0;
  double lookahead_distance = 0.5;
  for (size_t index = 0; index < waypoint_list.size(); index++) {
    double dx = waypoint_list[index][0] - robot_cartesian_state[0];
    double dy = waypoint_list[index][1] - robot_cartesian_state[3];
    if (std::sqrt(dx * dx + dy * dy) < lookahead_distance) {
      closest_index = index;
      std::cerr << std::sqrt(dx * dx + dy * dy) << std::endl;
    }
  }

  std::vector<CartesianPoint> truncated_waypoint_list;
  for (size_t index = closest_index; index < waypoint_list.size(); index++) {
    truncated_waypoint_list.push_back(waypoint_list[index]);
  }

  auto frenet_trajectory_planner = frenet_trajectory_planner::FrenetTrajectoryPlanner();
  auto planned_cartesian_trajectory = frenet_trajectory_planner.plan_by_waypoint(
    robot_cartesian_state,
    truncated_waypoint_list, 1.0);

  for (const auto & cartesian_state : planned_cartesian_trajectory) {
    std::cout << cartesian_state[0] << ", " << cartesian_state[3] << std::endl;
  }
  std::cout << "END" << std::endl;

  for (const auto & cartesian_point : waypoint_list) {
    std::cout << cartesian_point[0] << ", " << cartesian_point[1] << std::endl;
  }
  std::cout << "END" << std::endl;

  for (const auto & cartesian_point : truncated_waypoint_list) {
    std::cout << cartesian_point[0] << ", " << cartesian_point[1] << std::endl;
  }
  std::cout << "END" << std::endl;
  return 0;
}
