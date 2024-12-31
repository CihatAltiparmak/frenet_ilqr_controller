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

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

using DiffDriveRobotModelState = Vector3d;
using DiffDriveRobotModelInput = Vector2d;

class DiffDriveRobotModel : public Model<DiffDriveRobotModelState, DiffDriveRobotModelInput>
{
public:
  using StateT = DiffDriveRobotModelState;
  using InputT = DiffDriveRobotModelInput;
  DiffDriveRobotModel();
  Vector3d applySystemDynamics(const Vector3d & x, const Vector2d & u, const double dt) override;
  MatrixXd getStateMatrix(const Vector3d & x_eq, const Vector2d & u_eq, const double dt);
  MatrixXd getControlMatrix(const Vector3d & x_eq, const Vector2d & u_eq, const double dt);
};

}
