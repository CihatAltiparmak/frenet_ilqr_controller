#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <frenet_trajectory_planner/costs/lateral_distance_cost.hpp>
#include <frenet_trajectory_planner/costs/longtitutal_velocity_cost.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>

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

CartesianTrajectory FrenetTrajectoryPlanner::plan_by_waypoint(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list,
  const double max_time_per_point)
{
  auto frenet_frame_converter = std::make_shared<FrenetFrameConverter>();
  frenet_frame_converter->create_segments(waypoint_list);

  // robot_cartesian_state should start from first segment
  FrenetState robot_frenet_state =
    frenet_frame_converter->convert_cartesian2frenet_for_segment(robot_cartesian_state, 0);
  // const FrenetState & robot_frenet_state =
  //   frenet_frame_converter->convert_cartesian2frenet(robot_cartesian_state);

  std::cerr << "CONVERSION DBG: " << robot_frenet_state << std::endl;
  CartesianState robot_cartesian_state_tmp =
    frenet_frame_converter->convert_frenet2cartesian_for_segment(
    robot_frenet_state, 0);
  std::cerr << "CONVERSION DBG: " << robot_cartesian_state << std::endl;
  std::cerr << "CONVERSION DBG: " << robot_cartesian_state_tmp << std::endl;
  std::cerr << "CONVERSION DBG: END" << std::endl;
  // std::cout << "NAV2 JARBAY DEBUG: " << robot_cartesian_state << std::endl;
  // std::cout << "------------------" << std::endl;

  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(frenet_planner_config_);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    robot_frenet_state);

  auto frenet_trajectory_selector = FrenetTrajectorySelector(frenet_frame_converter);

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

  // {
  //   policies::AccelerationPolicyParameters parameters = {
  //     -20.0, // acceleration_min
  //     20.0   // acceleration_max
  //   };
  //   auto acceleration_policy = std::make_shared<policies::AccelerationPolicy>(
  //     parameters,
  //     frenet_frame_converter);
  //   frenet_trajectory_selector.add_policy(acceleration_policy);
  // }

  auto best_frenet_trajectory_optional = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories);

  auto best_frenet_trajectory = best_frenet_trajectory_optional.value();
  return frenet_frame_converter->convert_frenet2cartesian(best_frenet_trajectory);
}

}
