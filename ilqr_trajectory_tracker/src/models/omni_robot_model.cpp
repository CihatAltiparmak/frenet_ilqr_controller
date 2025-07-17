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

#include <ilqr_trajectory_tracker/models/omni_robot_model.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

OmniDriveRobotModel::OmniDriveRobotModel()
: Model<OmniDriveRobotModelState, OmniDriveRobotModelInput>()
{

}

OmniDriveRobotModelState OmniDriveRobotModel::applySystemDynamics(
  const OmniDriveRobotModelState & x, const OmniDriveRobotModelInput & u,
  const double dt)
{
  OmniDriveRobotModelState x_final;
  x_final <<
    x[0] + u[0] * std::cos(x[2]) * dt - u[1] * std::sin(x[2]) * dt,
    x[1] + u[0] * std::sin(x[2]) * dt + u[1] * std::cos(x[2]) * dt,
    x[2] + u[2] * dt;

  return x_final;
}

OmniDriveRobotModelInput OmniDriveRobotModel::applyLimits(const OmniDriveRobotModelInput & u) {
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

MatrixXd OmniDriveRobotModel::getStateMatrix(
  const OmniDriveRobotModelState & x_eq, const OmniDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix3d state_matrix;
  state_matrix << 
    1, 0, -u_eq[0] * std::sin(x_eq[2]) * dt - u_eq[1] * std::cos(x_eq[2]) * dt,
    0, 1, +u_eq[0] * std::cos(x_eq[2]) * dt - u_eq[1] * std::sin(x_eq[2]) * dt,
    0, 0, 1;

  return state_matrix;
}

MatrixXd OmniDriveRobotModel::getControlMatrix(
  const OmniDriveRobotModelState & x_eq, const OmniDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix<double, 3, 3> control_matrix;
  control_matrix << 
    std::cos(x_eq[2]) * dt, -std::sin(x_eq[2]) * dt, 0,
    std::sin(x_eq[2]) * dt,  std::cos(x_eq[2]) * dt, 0,
    0, 0, dt;

  return control_matrix;
}

}
