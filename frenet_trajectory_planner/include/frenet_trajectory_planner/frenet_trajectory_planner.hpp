#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
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
  CartesianTrajectory plan(
    const CartesianState & robot_cartesian_state,
    const std::vector<CartesianPoint> & waypoint_list,
    const double max_time_per_point);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

// TODO (CihatAltiparmak) : move the source parts of FrenetTrajectoryPlanner to cpp file. Now to move to cpp file throws out multiple definition error when built
FrenetTrajectoryPlanner::FrenetTrajectoryPlanner()
{
  frenet_planner_config_.min_lateral_distance = -1;
  frenet_planner_config_.max_lateral_distance = 1;
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

CartesianTrajectory FrenetTrajectoryPlanner::plan(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list,
  const double max_time_per_point)
{

  // assert there are at least 2 waypoint in waypoint list
  CartesianTrajectory planned_cartesian_trajectory = {robot_cartesian_state};
  for (int i = 0; i < waypoint_list.size() - 1; i++) {
    auto cartesian_trajectory = this->plan(
      planned_cartesian_trajectory.back(), waypoint_list[i], waypoint_list[i + 1]);

    planned_cartesian_trajectory.reserve(
      planned_cartesian_trajectory.size() + cartesian_trajectory.size());
    planned_cartesian_trajectory.insert(
      planned_cartesian_trajectory.end(),
      cartesian_trajectory.begin() + 1, cartesian_trajectory.end());
  }

  return planned_cartesian_trajectory;
}

}
