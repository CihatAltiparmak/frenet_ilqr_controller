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

#include <ilqr_trajectory_tracker/models/diff_robot_model.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

DiffDriveRobotModel::DiffDriveRobotModel()
: Model<DiffDriveRobotModelState, DiffDriveRobotModelInput>()
{

}

DiffDriveRobotModelState DiffDriveRobotModel::applySystemDynamics(
  const DiffDriveRobotModelState & x, const DiffDriveRobotModelInput & u,
  const double dt)
{
  DiffDriveRobotModelState x_final;
  x_final <<
    x[0] + u[0] * std::cos(x[2] + u[1] * dt) * dt,
    x[1] + u[0] * std::sin(x[2] + u[1] * dt) * dt,
    x[2] + u[1] * dt;

  return x_final;
}

DiffDriveRobotModelInput DiffDriveRobotModel::applyLimits(const DiffDriveRobotModelInput & u) {
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

MatrixXd DiffDriveRobotModel::getStateMatrix(
  const DiffDriveRobotModelState & x_eq, const DiffDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix3d state_matrix;
  state_matrix << 1, 0, -u_eq[0] * std::sin(x_eq[2]) * dt,
    0, 1, u_eq[0] * std::cos(x_eq[2]) * dt,
    0, 0, 1;

  return state_matrix;
}

MatrixXd DiffDriveRobotModel::getControlMatrix(
  const DiffDriveRobotModelState & x_eq, const DiffDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix<double, 3, 2> control_matrix;
  control_matrix << std::cos(x_eq[2]) * dt, 0,
    std::sin(x_eq[2]) * dt, 0,
    0, dt;

  return control_matrix;
}

}
