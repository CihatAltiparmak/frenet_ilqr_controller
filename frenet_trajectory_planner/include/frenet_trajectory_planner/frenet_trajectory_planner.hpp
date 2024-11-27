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

  CartesianTrajectory planByWaypoint(
    const CartesianState & robot_cartesian_state,
    const std::vector<CartesianPoint> & waypoint_list,
    const double max_time_per_point);

  void addPolicy(const std::shared_ptr<policies::Policy> & policy);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
  FrenetTrajectorySelector frenet_trajectory_selector_;
  std::shared_ptr<FrenetTrajectoryGenerator> frenet_trajectory_generator_;
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

  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector_.addCost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector_.addCost(longtitutal_velocity_cost_checker);
  }

  for (auto policy : selected_policies_) {
    frenet_trajectory_selector_.addPolicy(policy);
  }

  frenet_trajectory_generator_ =
    std::make_shared<FrenetTrajectoryGenerator>(frenet_planner_config_);
}

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner(
  const FrenetTrajectoryPlannerConfig & frenet_planner_config)
: frenet_planner_config_(frenet_planner_config)
{
  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector_.addCost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector_.addCost(longtitutal_velocity_cost_checker);
  }

  for (auto policy : selected_policies_) {
    frenet_trajectory_selector_.addPolicy(policy);
  }
}

CartesianTrajectory FrenetTrajectoryPlanner::planByWaypoint(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list,
  const double max_time_per_point)
{
  auto frenet_frame_converter = std::make_shared<FrenetFrameConverter>();
  frenet_frame_converter->createSegments(waypoint_list);

  frenet_trajectory_selector_.setFrenetFrameConverter(frenet_frame_converter);

  // robot_cartesian_state should start from first segment
  FrenetState robot_frenet_state =
    frenet_frame_converter->convertCartesian2FrenetForSegment(robot_cartesian_state, 0);

  FrenetTrajectory planned_frenet_trajectory = {};
  for (int i = 0; i < frenet_planner_config_.number_of_time_intervals; i++) {
    // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
    auto all_frenet_trajectories =
      frenet_trajectory_generator_->getAllPossibleFrenetTrajectories(
      robot_frenet_state);

    auto best_frenet_trajectory_optional =
      frenet_trajectory_selector_.selectBestFrenetTrajectory(
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
  return frenet_frame_converter->convertFrenet2Cartesian(planned_frenet_trajectory);
}

void FrenetTrajectoryPlanner::addPolicy(const std::shared_ptr<policies::Policy> & policy)
{
  selected_policies_.push_back(policy);
}

}
