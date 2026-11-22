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

#include <frenet_trajectory_planner/strategies/keep_velocity_strategy.hpp>
#include <iostream>

namespace frenet_trajectory_planner
{
namespace strategies
{

KeepVelocityStrategy::KeepVelocityStrategy(
  const FrenetTrajectoryPlannerConfig & frenet_planner_config)
: Strategy(frenet_planner_config)
{}

std::vector<FrenetTrajectory>
KeepVelocityStrategy::generateFrenetTrajectories(
  const FrenetState & f_state_initial, const size_t max_state_number)
{
  int max_t_inc = (frenet_planner_config_.max_time_interval -
    frenet_planner_config_.min_time_interval) / frenet_planner_config_.step_time_interval;
  int max_lon_inc = (frenet_planner_config_.max_longtitutal_velocity -
    frenet_planner_config_.min_longtitutal_velocity) /
    frenet_planner_config_.step_longtitutal_velocity;
  int max_lat_inc = (frenet_planner_config_.max_lateral_distance -
    frenet_planner_config_.min_lateral_distance) / frenet_planner_config_.step_lateral_distance;

  std::vector<FrenetTrajectory> frenet_trajectories(max_t_inc * max_lon_inc * max_lat_inc);
  #pragma omp parallel for
  for (int t = 0; t < max_t_inc; ++t) {
    for (int lon = 0; lon < max_lon_inc; ++lon) {
      for (int lat = 0; lat < max_lat_inc; ++lat) {
        StateLongtitutal state_longtitutal_final;
        state_longtitutal_final <<
          0,
          frenet_planner_config_.min_longtitutal_velocity + lon *
          frenet_planner_config_.step_longtitutal_velocity,
          0;

        StateLateral state_lateral_final;
        state_lateral_final <<
          frenet_planner_config_.min_lateral_distance + lat *
          frenet_planner_config_.step_lateral_distance,
          0,
          0;

        FrenetState f_state_final;
        f_state_final << state_longtitutal_final, state_lateral_final;
        auto frenet_trajectory = getFrenetTrajectory(
          f_state_initial, f_state_final,
          max_state_number,
            frenet_planner_config_.min_time_interval + t *
            frenet_planner_config_.step_time_interval);

        frenet_trajectories[t * max_lat_inc * max_lon_inc + lon * max_lat_inc +
          lat] = frenet_trajectory;
      }
    }
  }

  return frenet_trajectories;
}

FrenetTrajectory KeepVelocityStrategy::getFrenetTrajectory(
  const FrenetState & f_state_initial,
  const FrenetState & f_state_final,
  const size_t max_state_number,
  const double time_interval)
{
  FrenetTrajectory frenet_trajectory(max_state_number);

  StateLongtitutal lon_state_start = f_state_initial(seq(0, 2));
  StateLongtitutal lon_state_final = f_state_final(seq(0, 2));
  size_t lon_state_num = 0;
  frenet_trajectory[0]({0, 1, 2}) = lon_state_start;
  while (lon_state_num < max_state_number) {
    lon_state_start = frenet_trajectory[lon_state_num](seq(0, 2));
    auto longtitutal_velocity_planner = QuarticTrajectoryPlanner();
    if (!longtitutal_velocity_planner.setCoefficientsOrReturnFalse(
        lon_state_start[0], lon_state_start[1], lon_state_start[2],
        lon_state_final[1], lon_state_final[2],
        0, time_interval))
    {
      return {};
    }

    for (double t = frenet_planner_config_.dt; t <= time_interval; t += frenet_planner_config_.dt) {
      lon_state_num++;
      if (lon_state_num >= max_state_number) {
        break;
      }
      frenet_trajectory[lon_state_num][0] = longtitutal_velocity_planner.x(t);
      frenet_trajectory[lon_state_num][1] = longtitutal_velocity_planner.dx(t);
      frenet_trajectory[lon_state_num][2] = longtitutal_velocity_planner.ddx(t);
    }
  }

  StateLateral lat_state_start = f_state_initial(seq(3, 5));
  StateLateral lat_state_final = f_state_final(seq(3, 5));
  size_t lat_state_num = 0;
  frenet_trajectory[0](seq(3, 5)) = lat_state_start;
  while (lat_state_num < max_state_number) {
    lat_state_start = frenet_trajectory[lat_state_num](seq(3, 5));
    auto lateral_distance_planner = QuinticTrajectoryPlanner();
    if (!lateral_distance_planner.setCoefficientsOrReturnFalse(
        lat_state_start[0], lat_state_start[1], lat_state_start[2],
        lat_state_final[0], lat_state_final[1], lat_state_final[2],
        0, time_interval))
    {
      return {};
    }

    for (double t = frenet_planner_config_.dt; t <= time_interval; t += frenet_planner_config_.dt) {
      lat_state_num++;
      if (lat_state_num >= max_state_number) {
        break;
      }
      frenet_trajectory[lat_state_num][3] = lateral_distance_planner.x(t);
      frenet_trajectory[lat_state_num][4] = lateral_distance_planner.dx(t);
      frenet_trajectory[lat_state_num][5] = lateral_distance_planner.ddx(t);
    }
  }

  // std::cout << "**************" << std::endl;
  // for (auto fstate : frenet_trajectory) {
  //   std::cout
  //     << fstate[0] << ", "
  //     << fstate[1] << ", "
  //     << fstate[2] << ", "
  //     << fstate[3] << ", "
  //     << fstate[4] << ", "
  //     << fstate[5] << std::endl;
  // }
  // std::cout << "--------------" << std::endl;

  return frenet_trajectory;
}

}  // namespace strategies
}  // namespace frenet_trajectory_planner
