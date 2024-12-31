// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Tufan Kumbasar, Istanbul Technical University Artificial Intelligence and Intelligent Systems (AI2S) Laboratory
// Copyright (C) 2024 Prof. Behçet Uğur Töreyin
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

#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/costs/lateral_distance_cost.hpp>

int main()
{

  frenet_trajectory_planner::CartesianState robot_cartesian_state;
  frenet_trajectory_planner::CartesianPoint start_point;
  start_point << 1, 1;
  frenet_trajectory_planner::CartesianPoint final_point;
  final_point << 5, 1;
  frenet_trajectory_planner::FrenetState frenet_state_initial =
    frenet_trajectory_planner::FrenetState::Zero();
  frenet_state_initial[0] = 0;
  frenet_state_initial[1] = 1;
  frenet_state_initial[3] = 0.7;
  frenet_state_initial[4] = 0.5;

  frenet_trajectory_planner::FrenetTrajectoryPlannerConfig planner_config;
  planner_config.min_lateral_distance = -1;
  planner_config.max_lateral_distance = 1;
  planner_config.step_lateral_distance = 0.5;
  planner_config.min_longtitutal_velocity = 0;
  planner_config.max_longtitutal_velocity = 2;
  planner_config.step_longtitutal_velocity = 0.5;

  auto frenet_trajectory_generator = frenet_trajectory_planner::FrenetTrajectoryGenerator(
    planner_config);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    frenet_state_initial);

  auto frenet_trajectory_selector = frenet_trajectory_planner::FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<frenet_trajectory_planner::costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();
  auto frenet2cartesian_converter =
    frenet_trajectory_planner::Frenet2CartesianConverter<frenet_trajectory_planner::LineAdapter>(
    start_point, final_point);

  auto best_cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    best_frenet_trajectory);

  return 0;
}
