#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>

#include <memory>

namespace frenet_trajectory_planner
{

class FrenetTrajectoryPlanner
{
public:
  FrenetTrajectoryPlanner();
  FrenetTrajectoryPlanner(const FrenetTrajectoryPlannerConfig & frenet_trajectory_planner_config);
  CartesianTrajectory plan(
    const CartesianState & robot_cartesian_state,
    const CartesianPoint & start_point, const CartesianPoint & final_point);

  CartesianTrajectory planByWaypoint(
    const CartesianState & robot_cartesian_state,
    const std::vector<CartesianPoint> & waypoint_list);

  void addPolicy(const std::shared_ptr<policies::Policy> & policy);
  void addCost(const std::shared_ptr<costs::Cost> & cost);

  void setFrenetTrajectoryPlannerConfig(
    const FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config);

private:
  FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config_;
  FrenetTrajectorySelector frenet_trajectory_selector_;
  std::shared_ptr<FrenetTrajectoryGenerator> frenet_trajectory_generator_;
};

// TODO (CihatAltiparmak) : move the source parts of FrenetTrajectoryPlanner to cpp file. Now to move to cpp file throws out multiple definition error when built
FrenetTrajectoryPlanner::FrenetTrajectoryPlanner()
{
  frenet_trajectory_planner_config_.min_lateral_distance = -1;
  frenet_trajectory_planner_config_.max_lateral_distance = 1;
  frenet_trajectory_planner_config_.step_lateral_distance = 0.5;
  frenet_trajectory_planner_config_.min_longtitutal_velocity = 0;
  frenet_trajectory_planner_config_.max_longtitutal_velocity = 0.5;
  frenet_trajectory_planner_config_.step_longtitutal_velocity = 0.125;

  frenet_trajectory_generator_ =
    std::make_shared<FrenetTrajectoryGenerator>(frenet_trajectory_planner_config_);
}

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner(
  const FrenetTrajectoryPlannerConfig & frenet_trajectory_planner_config)
: frenet_trajectory_planner_config_(frenet_trajectory_planner_config)
{
  frenet_trajectory_generator_ =
    std::make_shared<FrenetTrajectoryGenerator>(frenet_trajectory_planner_config_);
}

CartesianTrajectory FrenetTrajectoryPlanner::planByWaypoint(
  const CartesianState & robot_cartesian_state,
  const std::vector<CartesianPoint> & waypoint_list)
{
  auto frenet_frame_converter = std::make_shared<FrenetFrameConverter>();
  frenet_frame_converter->createSegments(waypoint_list);

  frenet_trajectory_selector_.setFrenetFrameConverter(frenet_frame_converter);

  // robot_cartesian_state should start from first segment
  FrenetState robot_frenet_state =
    frenet_frame_converter->convertCartesian2FrenetForSegment(robot_cartesian_state, 0);

  FrenetTrajectory planned_frenet_trajectory = {robot_frenet_state};
  size_t remaining_state_number_ = frenet_trajectory_planner_config_.max_state_in_trajectory - 1;
  while (remaining_state_number_ > 0) {
    robot_frenet_state = planned_frenet_trajectory.back();

    // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
    auto all_frenet_trajectories =
      frenet_trajectory_generator_->getAllPossibleFrenetTrajectories(
      robot_frenet_state, remaining_state_number_);

    auto best_frenet_trajectory_optional =
      frenet_trajectory_selector_.selectBestFrenetTrajectory(
      all_frenet_trajectories);

    if (!best_frenet_trajectory_optional.has_value()) {
      break;
    }
    auto best_frenet_trajectory = best_frenet_trajectory_optional.value();
    if (best_frenet_trajectory.size() <= 1) {
      break;
    }

    planned_frenet_trajectory.insert(
      planned_frenet_trajectory.end(),
      std::next(best_frenet_trajectory.begin()), best_frenet_trajectory.end());

    remaining_state_number_ = frenet_trajectory_planner_config_.max_state_in_trajectory -
      planned_frenet_trajectory.size();
  }

  auto planned_cartesian_trajectory = frenet_frame_converter->convertFrenet2Cartesian(planned_frenet_trajectory);

  return planned_cartesian_trajectory;
}

void FrenetTrajectoryPlanner::addPolicy(const std::shared_ptr<policies::Policy> & policy)
{
  frenet_trajectory_selector_.addPolicy(policy);
}

void FrenetTrajectoryPlanner::addCost(const std::shared_ptr<costs::Cost> & cost)
{
  frenet_trajectory_selector_.addCost(cost);
}

void FrenetTrajectoryPlanner::setFrenetTrajectoryPlannerConfig(
  const FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config)
{
  frenet_trajectory_planner_config_ = frenet_trajectory_planner_config;
  frenet_trajectory_generator_ =
    std::make_shared<FrenetTrajectoryGenerator>(frenet_trajectory_planner_config_);
}

}
