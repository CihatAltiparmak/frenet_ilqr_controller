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

#include <ilqr_trajectory_tracker/models/omni_robot_model.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;  // NOLINT

namespace ilqr_trajectory_tracker
{

OmniRobotModel::OmniRobotModel()
: Model<5, 3>()
{
}

OmniRobotModel::StateT OmniRobotModel::applySystemDynamics(
  const StateT & x, const InputT & u,
  const double dt)
{
  StateT x_final;
  x_final <<
    x[0] + x[3] * dt,
    x[1] + x[4] * dt,
    x[2] + u[2] * dt,
    x[3] + u[0] * dt,
    x[4] + u[1] * dt;

  return x_final;
}

OmniRobotModel::InputT OmniRobotModel::applyLimits(const InputT & u)
{
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

OmniRobotModel::StateMatrixT OmniRobotModel::getStateMatrix(
  const StateT & x_eq, const InputT & u_eq,
  const double dt)
{
  StateMatrixT state_matrix;
  state_matrix <<
    1, 0, 0, dt, 0,
    0, 1, 0, 0, dt,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;

  return state_matrix;
}

OmniRobotModel::ControlMatrixT OmniRobotModel::getControlMatrix(
  const StateT & x_eq, const InputT & u_eq,
  const double dt)
{
  ControlMatrixT control_matrix;
  control_matrix <<
    0, 0, 0,
    0, 0, 0,
    0, 0, dt,
    dt, 0, 0,
    0, dt, 0;

  return control_matrix;
}

Vector3d OmniRobotModel::getTwistCommand(
  const StateT & x_initial,
  const InputT & u,
  const double dt
)
{
  Vector3d twist;
  // frenet generates velocity trajectories based on robot's frame.
  // however, we need to get vector according to the frame that corresponding state presents,
  // not just first state's frame.
  twist[0] = (x_initial[3] * std::cos(x_initial[2]) + x_initial[4] * std::sin(x_initial[2])) +
    (u[0] * std::cos(x_initial[2]) + u[1] * std::sin(x_initial[2])) * dt;
  twist[1] = (-x_initial[3] * std::sin(x_initial[2]) + x_initial[4] * std::cos(x_initial[4])) +
    (-u[0] * std::sin(x_initial[2]) + u[1] * std::cos(x_initial[2])) * dt;
  twist[2] = u[2];
  return twist;
}

OmniRobotModel::StateT
OmniRobotModel::fromFrenetCartesianState(
  const frenet_trajectory_planner::CartesianState & c_state)
{
  StateT x;
  x << c_state[0],
    c_state[3],
    c_state[6],
    c_state[1],
    c_state[4];
  return x;
}

}  // namespace ilqr_trajectory_tracker
