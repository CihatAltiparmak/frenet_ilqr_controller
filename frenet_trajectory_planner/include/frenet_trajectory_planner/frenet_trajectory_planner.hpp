#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <frenet_trajectory_planner/costs/lateral_distance_cost.hpp>
#include <frenet_trajectory_planner/costs/longtitutal_velocity_cost.hpp>

#include <memory>
#include <iostream>

namespace frenet_trajectory_planner
{

class FrenetTrajectoryPlanner
{
public:
  FrenetTrajectoryPlanner();
  FrenetTrajectoryPlanner(const FrenetTrajectoryPlannerConfig & frenet_planner_config);
  CartesianTrajectory plan(
    const CartesianState & robot_cartesian_state,
    const CartesianPoint & start_point, const CartesianPoint & final_point);

  CartesianTrajectory plan_alpha(
    const CartesianState & robot_cartesian_state,
    const CartesianPoint & start_point, const CartesianPoint & final_point);
  FrenetTrajectory plan_best_frenet_trajectory(const FrenetState & robot_frenet_state);

  std::vector<std::pair<double, std::unique_ptr<BaseAdapter>>> create_segments(
    const std::vector<CartesianPoint> & waypoint_list);

  CartesianTrajectory plan_by_waypoint(
    const CartesianState & robot_cartesian_state,
    const std::vector<CartesianPoint> & waypoint_list,
    const double max_time_per_point);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

// TODO (CihatAltiparmak) : move the source parts of FrenetTrajectoryPlanner to cpp file. Now to move to cpp file throws out multiple definition error when built
FrenetTrajectoryPlanner::FrenetTrajectoryPlanner()
{
  frenet_planner_config_.min_lateral_distance = -0;
  frenet_planner_config_.max_lateral_distance = 0;
  frenet_planner_config_.step_lateral_distance = 0.5;
  frenet_planner_config_.min_longtitutal_velocity = 0;
  frenet_planner_config_.max_longtitutal_velocity = 2;
  frenet_planner_config_.step_longtitutal_velocity = 0.5;
}

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner(
  const FrenetTrajectoryPlannerConfig & frenet_planner_config)
: frenet_planner_config_(frenet_planner_config)
{
}

CartesianTrajectory FrenetTrajectoryPlanner::plan(
  const CartesianState & robot_cartesian_state,
  const CartesianPoint & start_point,
  const CartesianPoint & final_point)
{
  auto frenet2cartesian_converter =
    Frenet2CartesianConverter<LineAdapter>(start_point, final_point);
  auto cartesian2frenet_converter =
    Cartesian2FrenetConverter<LineAdapter>(start_point, final_point);

  auto robot_frenet_state = cartesian2frenet_converter.convert_state(robot_cartesian_state);

  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(frenet_planner_config_);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    robot_frenet_state);

  auto frenet_trajectory_selector = FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector.add_cost(longtitutal_velocity_cost_checker);
  }

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();

  auto best_cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    best_frenet_trajectory);

  return best_cartesian_trajectory;
}

CartesianTrajectory FrenetTrajectoryPlanner::plan_alpha(
  const CartesianState & robot_cartesian_state,
  const CartesianPoint & start_point,
  const CartesianPoint & final_point)
{
  CartesianPoint ci;
  ci << 5, 5;
  double r = 5;
  Vector2d x_hat;
  x_hat << 0, -1;
  Vector2d y_hat;
  y_hat << 1, 0;
  double alpha = 3 * M_PI / 2;
  auto frenet2cartesian_converter =
    Frenet2CartesianConverter<CircleAdapter>(start_point, final_point, ci, r, x_hat, y_hat, alpha);
  auto cartesian2frenet_converter =
    Cartesian2FrenetConverter<CircleAdapter>(start_point, final_point, ci, r, x_hat, y_hat, alpha);

  auto robot_frenet_state = cartesian2frenet_converter.convert_state(robot_cartesian_state);
  std::cerr << "ROBOT FRENET STATE    : " << robot_frenet_state << std::endl;
  std::cerr << "ROBOT CARTESIAN STATE : " << frenet2cartesian_converter.convert_state(
    robot_frenet_state) << std::endl;
  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(frenet_planner_config_);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    robot_frenet_state);

  auto frenet_trajectory_selector = FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector.add_cost(longtitutal_velocity_cost_checker);
  }

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();

  for (auto fstate : best_frenet_trajectory) {
    std::cerr << "FSTATE : " << fstate[0] << " | " << fstate[3] << std::endl;
  }

  auto best_cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    best_frenet_trajectory);

  return best_cartesian_trajectory;
}

FrenetTrajectory FrenetTrajectoryPlanner::plan_best_frenet_trajectory(
  const FrenetState & robot_frenet_state)
{
  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(frenet_planner_config_);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    robot_frenet_state);

  auto frenet_trajectory_selector = FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector.add_cost(longtitutal_velocity_cost_checker);
  }

  std::cerr << "all_frenet_trajectory : " << all_frenet_trajectories.size() << std::endl;
  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();

  return best_frenet_trajectory;
}

std::vector<std::pair<double, std::unique_ptr<BaseAdapter>>>
FrenetTrajectoryPlanner::create_segments(const std::vector<CartesianPoint> & waypoint_list)
{

  std::vector<std::pair<double, std::unique_ptr<BaseAdapter>>> segments;
  CartesianPoint last_waypoint = waypoint_list[0];
  for (auto waypoint_it = waypoint_list.begin() + 1; (waypoint_it + 1) != waypoint_list.end();
    waypoint_it++)
  {
    CartesianPoint qim1 = last_waypoint;
    CartesianPoint qi = *(waypoint_it);
    CartesianPoint qip1 = *(waypoint_it + 1);

    Vector2d yi = (qi - qim1) / (qi - qim1).norm();
    Vector2d yip1 = (qip1 - qi) / (qip1 - qi).norm();
    double alphai = std::acos(yi.dot(yip1));

    double delta = 1;
    double li = std::min(
      {(qi - qim1).norm() / 2,
        (qip1 - qi).norm() / 2,
        delta * std::sin(alphai / 2) / (1 - std::cos(alphai / 2))});
    double ri = li / std::tan(alphai / 2);
    CartesianPoint ci = qi + ((yip1 - yi) / (yip1 - yi).norm()) * (ri / std::cos(alphai / 2));
    Vector2d xi = (qi - li * yi - ci) / (qi - li * yi - ci).norm();

    double chross = yip1.x() * yi.y() - yip1.y() * yi.x();
    int sign_indicactor = (chross > 0) ? 1 : -1;

    CartesianPoint new_q1 = qim1;
    CartesianPoint new_q2 = ci + ri * xi;
    CartesianPoint new_q3 = ci + ri * (xi * std::cos(alphai) + yi * std::sin(alphai));
    CartesianPoint new_q4 = qip1;

    std::cout << new_q1[0] << ", " << new_q1[1] << std::endl;
    std::cout << new_q2[0] << ", " << new_q2[1] << std::endl;

    segments.push_back(
      {(new_q2 - new_q1).norm(), std::make_unique<LineAdapter>(new_q1, new_q2)}
    );

    segments.push_back(
      {alphai * ri,
        std::make_unique<CircleAdapter>(new_q2, new_q3, ci, ri, xi, yi, alphai, sign_indicactor)}
    );

    last_waypoint = new_q3;
  }

  std::cout << waypoint_list.back()[0] << ", " << waypoint_list.back()[1] << std::endl;
  segments.push_back(
    {(waypoint_list.back() - last_waypoint).norm(),
      std::make_unique<LineAdapter>(last_waypoint, waypoint_list.back())}
  );


  // if (waypoint_list.size() == 2) {
  //   segments.push_back(
  //     {(waypoint_list[1] - waypoint_list[0]).norm(), std::make_unique<LineAdapter>(waypoint_list[0], waypoint_list[1])}
  //   );
  // }
  return segments;
}

CartesianTrajectory FrenetTrajectoryPlanner::plan_by_waypoint(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list,
  const double max_time_per_point)
{
  // std::vector<std::pair<double, std::unique_ptr<BaseAdapter>>> segments;

  // for (auto waypoint_it = waypoint_list.begin() + 1; (waypoint_it + 1) != waypoint_list.end(); waypoint_it++) {
  //   CartesianPoint qim1 = *(waypoint_it - 1);
  //   CartesianPoint qi   = *(waypoint_it);
  //   CartesianPoint qip1 = *(waypoint_it + 1);

  //   Vector2d yi   = (qi - qim1) / (qi - qim1).norm();
  //   Vector2d yip1 = (qip1 - qi) / (qip1 - qi).norm();
  //   double alphai = std::acos(yi.dot(yip1));

  //   double delta = 1;
  //   double li = std::min(
  //     {(qi - qim1).norm() / 2,
  //     (qip1 - qi).norm() / 2,
  //     delta * std::sin(alphai / 2) / (1 - std::cos(alphai / 2))});
  //   double ri = li / std::tan(alphai / 2);
  //   CartesianPoint ci = qi + ((yip1 - yi) / (yip1 - yi).norm()) * (ri / std::cos(alphai / 2));
  //   Vector2d xi = (qi - li * yi - ci) / (qi - li * yi - ci).norm();
  //   // std::cerr << "chan lee : " << xi << std::endl;

  //   // ri = 10;
  //   CartesianPoint new_q1 = qim1;
  //   CartesianPoint new_q2 = ci + ri * xi;
  //   CartesianPoint new_q3 = ci + ri * (xi * std::cos(alphai) + yi * std::sin(alphai));
  //   CartesianPoint new_q4 = qip1;
  //   // std::cerr << "segments : " << std::endl
  //   //           << new_q1 << std::endl
  //   //           << "-------" << std::endl
  //   //           << new_q2 << std::endl
  //   //           << "-------" << std::endl
  //   //           << new_q3 << std::endl
  //   //           << "-------" << std::endl
  //   //           << new_q4 << std::endl;
  //   std::cout << new_q1[0] << ", " << new_q1[1] << std::endl;
  //   std::cout << new_q2[0] << ", " << new_q2[1] << std::endl;
  //   std::cout << new_q3[0] << ", " << new_q3[1] << std::endl;
  //   std::cout << new_q4[0] << ", " << new_q4[1] << std::endl;

  //   segments.push_back(
  //     {(new_q2 - new_q1).norm(), std::make_unique<LineAdapter>(new_q1, new_q2)}
  //   );

  //   segments.push_back(
  //     {alphai * ri, std::make_unique<CircleAdapter>(new_q2, new_q3, ci, ri, xi, yi, alphai)}
  //   );


  //   // segments.push_back(
  //   //   {(new_q4 - new_q3).norm(), std::make_unique<LineAdapter>(new_q3, new_q4)}
  //   // );
  // }

  // if (waypoint_list.size() == 2) {
  //   segments.push_back(
  //     {(waypoint_list[1] - waypoint_list[0]).norm(), std::make_unique<LineAdapter>(waypoint_list[0], waypoint_list[1])}
  //   );
  // }

  auto segments = create_segments(waypoint_list);

  std::cout << "END" << std::endl;
  const FrenetState & robot_frenet_state = segments[0].second->convert_cartesian2frenet(
    robot_cartesian_state);

  auto best_frenet_trajectory = this->plan_best_frenet_trajectory(robot_frenet_state);

  double current_longitutal_length = 0;
  size_t current_segment_index = 0;
  CartesianTrajectory cartesian_trajectory;
  for (auto frenet_state : best_frenet_trajectory) {
    // std::cerr << "arclength : " << current_longitutal_length + segments[current_segment_index].first << " | " << frenet_state[0] << " | " << frenet_state[3] << std::endl;
    // std::cerr << "segments: " << current_segment_index << " | " << segments.size() << std::endl;
    if (frenet_state[0] < current_longitutal_length + segments[current_segment_index].first) {
      FrenetState converted_frenet_state = frenet_state;
      converted_frenet_state[0] -= current_longitutal_length;
      CartesianState cartesian_state =
        segments[current_segment_index].second->convert_frenet2cartesian(converted_frenet_state);
      cartesian_trajectory.push_back(cartesian_state);
    } else {
      current_longitutal_length += segments[current_segment_index].first;
      current_segment_index++;
    }

    if (current_segment_index >= segments.size()) {
      break;
    }
  }
  // std::cout << cartesian_trajectory.size() << " | " << best_frenet_trajectory.size() << std::endl;
  return cartesian_trajectory;
}

}
