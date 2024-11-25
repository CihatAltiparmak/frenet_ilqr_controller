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

  void add_policy(const std::shared_ptr<policies::Policy> & policy);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
  std::vector<std::shared_ptr<policies::Policy>> selected_policies_;
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

  for (auto policy : selected_policies_) {
    frenet_trajectory_selector.add_policy(policy);
  }

  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(frenet_planner_config_);

  // robot_cartesian_state should start from first segment
  FrenetState robot_frenet_state =
    frenet_frame_converter->convert_cartesian2frenet_for_segment(robot_cartesian_state, 0);

  FrenetTrajectory planned_frenet_trajectory = {};
  for (int i = 0; i < frenet_planner_config_.number_of_time_intervals; i++) {
    // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
    auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
      robot_frenet_state);

    auto best_frenet_trajectory_optional = frenet_trajectory_selector.select_best_frenet_trajectory(
      all_frenet_trajectories);

    if (!best_frenet_trajectory_optional.has_value()) {
      break;
    }
    auto best_frenet_trajectory = best_frenet_trajectory_optional.value();
    planned_frenet_trajectory.insert(
      planned_frenet_trajectory.end(),
      best_frenet_trajectory.begin(), best_frenet_trajectory.end());

    robot_frenet_state = planned_frenet_trajectory.back();
  }
  return frenet_frame_converter->convert_frenet2cartesian(planned_frenet_trajectory);
}

void FrenetTrajectoryPlanner::add_policy(const std::shared_ptr<policies::Policy> & policy)
{
  selected_policies_.push_back(policy);
}

}
