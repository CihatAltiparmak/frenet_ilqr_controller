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

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();

  return best_frenet_trajectory;
}

CartesianTrajectory FrenetTrajectoryPlanner::plan_by_waypoint(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list,
  const double max_time_per_point)
{
  auto frenet_frame_converter = FrenetFrameConverter();
  frenet_frame_converter.create_segments(waypoint_list);

  // robot_cartesian_state should start from first segment
  const FrenetState & robot_frenet_state =
    frenet_frame_converter.convert_cartesian2frenet_for_segment(robot_cartesian_state, 0);

  // TODO (CihatAltiparmak) : Remove this method and use FrenetFrameConverter instance to eliminate
  // unfeasible trajectories
  auto best_frenet_trajectory = this->plan_best_frenet_trajectory(robot_frenet_state);

  return frenet_frame_converter.convert_frenet2cartesian(best_frenet_trajectory);
}

}
