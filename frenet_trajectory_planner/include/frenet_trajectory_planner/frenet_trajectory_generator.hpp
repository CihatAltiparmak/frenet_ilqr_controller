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

#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>
#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <memory>

using namespace Eigen;

namespace frenet_trajectory_planner
{

enum Behavior {
  KEEP_VELOCITY,
  MERGE_AND_THEN_STOP
};

class FrenetTrajectoryGenerator
{
public:
  FrenetTrajectoryGenerator(const FrenetTrajectoryPlannerConfig & frenet_planner_config);

  std::vector<FrenetTrajectory> getAllPossibleFrenetTrajectories(
    const FrenetState & frenet_state_initial, 
    const size_t max_state_number,
    const Behavior & selected_behavior);

  FrenetTrajectory getFrenetTrajectory(
    const FrenetState & frenet_state_initial,
    const FrenetState & frenet_state_final,
    const size_t max_state_number,
    const Behavior & selected_behavior);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

}
