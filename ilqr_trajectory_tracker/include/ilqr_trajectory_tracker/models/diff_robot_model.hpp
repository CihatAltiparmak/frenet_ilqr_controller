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

#include <ilqr_trajectory_tracker/models/base_model.hpp>

#include <Eigen/Dense>
#include <cmath>
#include "frenet_trajectory_planner/type_definitions.hpp"

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

using DiffDriveRobotModelState = Vector4d;
using DiffDriveRobotModelInput = Vector2d;

class DiffDriveRobotModel : public Model<4, 2>
{
public:
  DiffDriveRobotModel();
  StateT applySystemDynamics(const StateT & x, const InputT & u, const double dt) override;
  InputT applyLimits(const InputT & u) override;
  StateMatrixT getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  ControlMatrixT getControlMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  Vector2d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt);
  
  static StateT fromFrenetCartesianState(const frenet_trajectory_planner::CartesianState & c_state);
};

}
