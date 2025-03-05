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

#include <ilqr_trajectory_tracker/models/ackermann_robot_model.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

AckermannRobotModel::AckermannRobotModel(const double wheelbase)
: Model<AckermannRobotModelState, AckermannRobotModelInput>(), wheelbase_(wheelbase)
{

}

AckermannRobotModelState AckermannRobotModel::applySystemDynamics(
  const AckermannRobotModelState & x, const AckermannRobotModelInput & u,
  const double dt)
{
  AckermannRobotModelState x_final;
  x_final <<
    x[0] + u[0] * std::cos(x[2]) * dt,
    x[1] + u[0] * std::sin(x[2]) * dt,
    x[2] + (u[0] / wheelbase_) * std::tan(u[1]) * dt;

  return x_final;
}

AckermannRobotModelInput AckermannRobotModel::applyLimits(const AckermannRobotModelInput & u) {
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

MatrixXd AckermannRobotModel::getStateMatrix(
  const AckermannRobotModelState & x_eq, const AckermannRobotModelInput & u_eq,
  const double dt)
{
  Matrix3d state_matrix;
  state_matrix << 1, 0, -u_eq[0] * std::sin(x_eq[2]) * dt,
    0, 1, u_eq[0] * std::cos(x_eq[2]) * dt,
    0, 0, 1;

  return state_matrix;
}

MatrixXd AckermannRobotModel::getControlMatrix(
  const AckermannRobotModelState & x_eq, const AckermannRobotModelInput & u_eq,
  const double dt)
{
  Matrix<double, 3, 2> control_matrix;
  control_matrix << std::cos(x_eq[2]) * dt, 0,
    std::sin(x_eq[2]) * dt, 0,
    std::tan(u_eq[1]) / wheelbase_ * dt, (u_eq[0] / wheelbase_) * (1 / (std::cos(u_eq[1]) * std::cos(u_eq[1]))) * dt;

  return control_matrix;
}

}
