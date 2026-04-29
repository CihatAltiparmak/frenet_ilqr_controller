<<<<<<< HEAD
=======
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

>>>>>>> 4094837 (Format code according to ros standard (#71))
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>
#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>

using namespace Eigen;  // NOLINT

namespace frenet_trajectory_planner
{

class FrenetTrajectoryGenerator
{
public:
  explicit FrenetTrajectoryGenerator(
    const FrenetTrajectoryPlannerConfig & frenet_planner_config);

  std::vector<FrenetTrajectory> getAllPossibleFrenetTrajectories(
    const FrenetState & frenet_state_initial, const size_t max_state_number);

  FrenetTrajectory getFrenetTrajectory(
    const FrenetState & frenet_state_initial,
    const FrenetState & frenet_state_final,
    const size_t max_state_number);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

}  // namespace frenet_trajectory_planner
