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
: Model<4, 2>()
{

}

DiffDriveRobotModel::StateT DiffDriveRobotModel::applySystemDynamics(
  const StateT & x, const InputT & u,
  const double dt)
{
  StateT x_final;
  x_final <<
    x[0] + x[3] * std::cos(x[2]) * dt,
    x[1] + x[3] * std::sin(x[2]) * dt,
    x[2] + u[1] * dt,
    x[3] + u[0] * dt;

  return x_final;
}

DiffDriveRobotModel::InputT DiffDriveRobotModel::applyLimits(const InputT & u) {
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

DiffDriveRobotModel::StateMatrixT DiffDriveRobotModel::getStateMatrix(
  const StateT & x_eq, const InputT & u_eq,
  const double dt)
{
  StateMatrixT state_matrix;
  state_matrix << 
    1, 0, -x_eq[3] * std::sin(x_eq[2]) * dt, std::cos(x_eq[2]) * dt,
    0, 1, +x_eq[3] * std::cos(x_eq[2]) * dt, std::sin(x_eq[2]) * dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  return state_matrix;
}

DiffDriveRobotModel::ControlMatrixT DiffDriveRobotModel::getControlMatrix(
  const StateT & x_eq, const InputT & u_eq,
  const double dt)
{
  ControlMatrixT control_matrix;
  control_matrix <<
    0, 0,
    0, 0,
    0, dt,
    dt, 0;

  return control_matrix;
}

Vector2d DiffDriveRobotModel::getTwistCommand(
  const StateT & x_initial,
  const InputT & u,
  const double dt
)
{
  Vector2d twist;
  // velocity_new = velocity_robot + acceleration * dt
  twist[0] = x_initial[3] + u[0] * dt;

  twist[1] = u[1];
  return twist;
}

DiffDriveRobotModel::StateT
DiffDriveRobotModel::fromFrenetCartesianState(
  const frenet_trajectory_planner::CartesianState & c_state)
{
  StateT x;
  // TODO (CihatAltiparmak) : find the signs of the velocitites as well
  double vel = std::hypot(c_state[1], c_state[4]);
  x << c_state[0],
       c_state[3],
       c_state[6],
       vel;
  return x;
}

}
