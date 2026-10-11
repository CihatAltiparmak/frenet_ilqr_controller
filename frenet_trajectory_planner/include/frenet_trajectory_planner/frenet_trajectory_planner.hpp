// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Dr. Tufan Kumbasar, ITU AI2S Lab
// Copyright (C) 2024 Prof. Dr. Behçet Uğur Töreyin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/strategies/keep_velocity_strategy.hpp>
#include <frenet_trajectory_planner/strategies/merge_and_stop_strategy.hpp>
#include "angles/angles.h"

namespace frenet_trajectory_planner
{

class FrenetTrajectoryPlanner
{
public:
  FrenetTrajectoryPlanner();
  explicit FrenetTrajectoryPlanner(
    const FrenetTrajectoryPlannerConfig & frenet_trajectory_planner_config);
  CartesianTrajectory plan(
    const CartesianState & robot_cartesian_state,
    const CartesianPoint & start_point, const CartesianPoint & final_point);

  CartesianTrajectory planByWaypoint(
    const CartesianState & robot_cartesian_state,
    const std::vector<CartesianPoint> & waypoint_list,
    std::shared_ptr<DebugInfo> debug_info);

  void makeYawTrajectoryFeasible(
    const CartesianState & c_state_robot,
    CartesianTrajectory & c_trajectory);

  void addPolicy(const std::shared_ptr<policies::Policy> & policy);
  void addCost(const std::shared_ptr<costs::Cost> & cost);

  void setFrenetTrajectoryPlannerConfig(
    const FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config);

  bool shouldMergeAndStop(
    const FrenetState f_state, double arclength, double time_interval);

private:
  FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config_;
  FrenetTrajectorySelector frenet_trajectory_selector_;
  std::shared_ptr<FrenetTrajectoryGenerator> frenet_trajectory_generator_;
};

// TODO(CihatAltiparmak) : move the source parts of FrenetTrajectoryPlanner to cpp file.
// Now to move to cpp file throws out multiple definition error when built
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
  const std::vector<CartesianPoint> & waypoint_list,
  std::shared_ptr<DebugInfo> debug_info)
{
  auto frenet_frame_converter = std::make_shared<FrenetFrameConverter>();
  frenet_frame_converter->createSegments(waypoint_list);

  frenet_trajectory_selector_.setFrenetFrameConverter(frenet_frame_converter);

  Info info;
  info.arclength = frenet_frame_converter->getArclength();

  // robot_cartesian_state should start from first segment
  FrenetState robot_frenet_state =
    frenet_frame_converter->convertCartesian2FrenetForSegment(robot_cartesian_state, 0);

  FrenetTrajectory planned_frenet_trajectory;
  planned_frenet_trajectory.push_back(robot_frenet_state);

  std::string current_behavior = "keep_velocity";
  size_t remaining_state_number_ = frenet_trajectory_planner_config_.max_state_in_trajectory;
  while (remaining_state_number_ > 0) {
    // TODO(CihatAltiparmak) : eliminate some trajectories in frenet level
    std::vector<FrenetTrajectory> all_frenet_trajectories;
    if (current_behavior == "keep_velocity") {
      frenet_trajectory_generator_->setStrategy(
        std::make_unique<strategies::KeepVelocityStrategy>(
          frenet_trajectory_planner_config_));
      all_frenet_trajectories =
        frenet_trajectory_generator_->getAllPossibleFrenetTrajectories(
        robot_frenet_state, remaining_state_number_);
    } else {
      frenet_trajectory_generator_->setStrategy(
        std::make_unique<strategies::MergeAndStopStrategy>(
          frenet_trajectory_planner_config_, info.arclength));
      all_frenet_trajectories =
        frenet_trajectory_generator_->getAllPossibleFrenetTrajectories(
        robot_frenet_state, remaining_state_number_);
    }

    auto best_frenet_trajectory_optional =
      frenet_trajectory_selector_.selectBestFrenetTrajectory(
      all_frenet_trajectories, info, debug_info);

    if (!best_frenet_trajectory_optional.has_value()) {
      break;
    }
    auto best_frenet_trajectory = best_frenet_trajectory_optional.value();
    if (best_frenet_trajectory.size() <= 1) {
      break;
    }

    for (size_t i = 1; i < best_frenet_trajectory.size(); ++i) {
      const auto & f_state = best_frenet_trajectory[i];
      planned_frenet_trajectory.push_back(f_state);
      if (shouldMergeAndStop(f_state, info.arclength,
          frenet_trajectory_planner_config_.max_time_interval))
      {
        current_behavior = "merge_and_stop";
        break;
      }
    }

    remaining_state_number_ = frenet_trajectory_planner_config_.max_state_in_trajectory -
      planned_frenet_trajectory.size();
    robot_frenet_state = planned_frenet_trajectory.back();
  }

  auto planned_cartesian_trajectory =
    frenet_frame_converter->convertFrenet2Cartesian(planned_frenet_trajectory);

  planned_cartesian_trajectory[0] = robot_cartesian_state;
  // arrange yaw to make it feasible to follow by iterative lqr
  makeYawTrajectoryFeasible(robot_cartesian_state, planned_cartesian_trajectory);

  return planned_cartesian_trajectory;
}

bool FrenetTrajectoryPlanner::shouldMergeAndStop(
  const FrenetState f_state, double arclength, double time_interval)
{
  auto longtitutal_distance_planner = QuarticTrajectoryPlanner();
  if (!longtitutal_distance_planner.setCoefficientsOrReturnFalse(
      f_state[0], f_state[1], f_state[2],
      0.0, 0.0,
      0, time_interval))
  {
    return false;
  }

  return longtitutal_distance_planner.x(time_interval) >= arclength;
}

void FrenetTrajectoryPlanner::makeYawTrajectoryFeasible(
  const CartesianState & c_state_robot,
  CartesianTrajectory & c_trajectory)
{
  if (c_trajectory.empty()) {
    return;
  }

  double yaw_diff_min = angles::shortest_angular_distance(
      c_state_robot[6], c_trajectory[0][6]);
  c_trajectory[0][6] = c_state_robot[6] + yaw_diff_min;
  for (size_t i = 1; i < c_trajectory.size(); ++i) {
    double yaw_diff_min = angles::shortest_angular_distance(
      c_trajectory[i - 1][6], c_trajectory[i][6]);
    c_trajectory[i][6] = c_trajectory[i - 1][6] + yaw_diff_min;
  }
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

}  // namespace frenet_trajectory_planner
