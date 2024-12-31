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

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

using StateLateral = Vector3d;
using StateLongtitutal = Vector3d;
using FrenetState = Vector<double, 6>; // s, s_dot, s_dot_dot, d, d_dot, d_dot_dot
using FrenetTrajectory = std::vector<FrenetState>;

using CartesianState = Vector<double, 7>; // x, x_dot, x_dot_dot, y, y_dot, y_dot_dot, yaw
using CartesianTrajectory = std::vector<CartesianState>;

using CartesianPoint = Vector2d;

typedef struct FrenetTrajectoryPlannerConfig
{
  double min_lateral_distance;
  double max_lateral_distance;
  double step_lateral_distance;
  double min_longtitutal_velocity;
  double max_longtitutal_velocity;
  double step_longtitutal_velocity;
  double time_interval = 1; // 100;
  int max_state_in_trajectory = 40;
  double dt = 0.05; // time discretization
} FrenetTrajectoryPlannerConfig;

}
