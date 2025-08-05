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

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <iostream>

namespace frenet_trajectory_planner
{

FrenetTrajectoryGenerator::FrenetTrajectoryGenerator(
  const FrenetTrajectoryPlannerConfig & frenet_planner_config)
: frenet_planner_config_(frenet_planner_config)
{

  std::cout << "I DONT CARE ABOUT THE VIP : " << frenet_planner_config.max_longtitutal_velocity << " | " << frenet_planner_config_.max_longtitutal_velocity << std::endl;
}

std::vector<FrenetTrajectory> FrenetTrajectoryGenerator::getAllPossibleFrenetTrajectories(
  const FrenetState & frenet_state_initial, size_t max_state_number)
{

  std::cout << "XXXXXXXXXXXXXX: " << frenet_planner_config_.max_longtitutal_velocity << std::endl;
  std::vector<FrenetTrajectory> frenet_trajectories;
  for (double longtitutal_velocity_final = frenet_planner_config_.min_longtitutal_velocity;
    longtitutal_velocity_final <= frenet_planner_config_.max_longtitutal_velocity;
    longtitutal_velocity_final += frenet_planner_config_.step_longtitutal_velocity)
  {
    for (double lateral_distance_final = frenet_planner_config_.min_lateral_distance;
      lateral_distance_final <= frenet_planner_config_.max_lateral_distance;
      lateral_distance_final += frenet_planner_config_.step_lateral_distance)
    {
      StateLongtitutal state_longtitutal_final;
      state_longtitutal_final << 0, longtitutal_velocity_final, 0;

      StateLateral state_lateral_final;
      state_lateral_final << lateral_distance_final, 0, 0;

      FrenetState frenet_state_final;
      frenet_state_final << state_longtitutal_final, state_lateral_final;
      auto frenet_trajectory = getFrenetTrajectory(
        frenet_state_initial, frenet_state_final,
        max_state_number);
      if (!frenet_trajectory.empty()) {
        frenet_trajectories.push_back(frenet_trajectory);
      }
    }
  }

  return frenet_trajectories;
}

FrenetTrajectory FrenetTrajectoryGenerator::getFrenetTrajectory(
  const FrenetState & frenet_state_initial,
  const FrenetState & frenet_state_final, const size_t max_state_number)
{
  auto longtitual_state_initial = frenet_state_initial(seq(0, 2));
  auto longtitual_state_final = frenet_state_final(seq(0, 2));
  auto longtitutal_velocity_planner = QuarticTrajectoryPlanner();
  if (!longtitutal_velocity_planner.setCoefficientsOrReturnFalse(
      longtitual_state_initial[0], longtitual_state_initial[1], longtitual_state_initial[2],
      longtitual_state_final[1], longtitual_state_final[2],
      0, frenet_planner_config_.time_interval))
  {
    return {};
  }

  auto lateral_state_initial = frenet_state_initial(seq(3, 5));
  auto lateral_state_final = frenet_state_final(seq(3, 5));
  auto lateral_distance_planner = QuinticTrajectoryPlanner();
  if (!lateral_distance_planner.setCoefficientsOrReturnFalse(
      lateral_state_initial[0], lateral_state_initial[1], lateral_state_initial[2],
      lateral_state_final[0], lateral_state_final[1], lateral_state_final[2],
      0, frenet_planner_config_.time_interval))
  {
    return {};
  }

  FrenetTrajectory frenet_trajectory;

  // assert dt is smaller than time_interval
  double maximum_time_interval = std::min(
    frenet_planner_config_.dt * max_state_number,
    frenet_planner_config_.time_interval);
  for (double t = 0; t <= maximum_time_interval; t += frenet_planner_config_.dt) {
    StateLongtitutal state_longtitutal;
    state_longtitutal[0] = longtitutal_velocity_planner.x(t);
    state_longtitutal[1] = longtitutal_velocity_planner.dx(t);
    state_longtitutal[2] = longtitutal_velocity_planner.ddx(t);

    StateLateral state_lateral;
    state_lateral[0] = lateral_distance_planner.x(t);
    state_lateral[1] = lateral_distance_planner.dx(t);
    state_lateral[2] = lateral_distance_planner.ddx(t);

    FrenetState frenet_state;
    frenet_state << state_longtitutal, state_lateral;
    frenet_trajectory.push_back(frenet_state);
  }

  return frenet_trajectory;
}

}
