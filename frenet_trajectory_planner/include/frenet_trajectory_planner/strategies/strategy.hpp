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

#include <vector>
#include <frenet_trajectory_planner/type_definitions.hpp>

namespace frenet_trajectory_planner
{
namespace strategies
{
class Strategy {
public:
  Strategy(
    const FrenetTrajectoryPlannerConfig & frenet_planner_config)
  {
    frenet_planner_config_ = frenet_planner_config;
  }

  virtual ~Strategy() = default;
  virtual std::vector<FrenetTrajectory> generateFrenetTrajectories(
    const FrenetState & f_state_initial, const size_t max_state_number) = 0;
  virtual FrenetTrajectory getFrenetTrajectory(
    const FrenetState & f_state_initial,
    const FrenetState & f_state_final,
    const size_t max_state_number,
    const double time_interval) = 0;

protected:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

}  // namespace strategies
}  // namespace frenet_trajectory_planner
