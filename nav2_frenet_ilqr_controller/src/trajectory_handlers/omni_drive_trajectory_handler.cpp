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

#include "nav2_frenet_ilqr_controller/trajectory_handlers/omni_drive_trajectory_handler.hpp"

#include "ilqr_trajectory_tracker/models/omni_robot_model.hpp"
#include "ilqr_trajectory_tracker/ilqr_optimizer.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace trajectory_handlers
{

using frenet_trajectory_planner::CartesianState;
using frenet_trajectory_planner::CartesianTrajectory;
using ilqr_trajectory_tracker::OmniRobotModel;

OmniDriveTrajectoryHandler::OmniDriveTrajectoryHandler(const Parameters & params)
: TrajectoryHandler(params)
{
}

Vector3d OmniDriveTrajectoryHandler::processTrajectory(
  const CartesianState & c_state_robot,
  const CartesianTrajectory & c_trajectory_robot)
{
  if (c_trajectory_robot.empty()) {
    throw std::runtime_error("There is no trajectory to be tracked!");
  }

  ilqr_trajectory_tracker::NewtonOptimizer<OmniRobotModel> newton_optimizer;

  auto x_robot = OmniRobotModel::fromFrenetCartesianState(c_state_robot);
  auto X_feasible = newton_optimizer.fromFrenetCartesianTrajectory(c_trajectory_robot);
  // TODO(CihatAltiparmak) : add behavior mode into frenet_trajectory_planner.
  // The velocity trajectory
  // can be planned using Quinctic Polynom instead of Quartic Polynom  which takes into account
  // the finishing point as well
  // If the robot is to approach to the goal, tell ILQR to deccelerate
  // by filling velocity states by zero and keep the goal's x, y and yaw angle states same
  size_t state_number_to_track =
    params_.frenet_trajectory_planner_config.max_state_in_trajectory - 1;
  if (X_feasible.size() < state_number_to_track) {
    size_t state_number_for_stopping = state_number_to_track - X_feasible.size();
    OmniRobotModel::StateT x_stop = X_feasible.back();
    x_stop[3] = 0.0;
    x_stop[4] = 0.0;
    for (size_t i = 0; i < state_number_for_stopping; ++i) {
      X_feasible.push_back(x_stop);
    }
  }

  newton_optimizer.setIterationNumber(params_.iteration_number);
  newton_optimizer.setInputConstraints(params_.input_limits_min, params_.input_limits_max);
  auto U_optimal = newton_optimizer.optimize(
    x_robot, X_feasible, params_.Q, params_.R,
    params_.time_discretization);

  if (U_optimal.empty()) {
    throw std::runtime_error("Iterative LQR couldn't find any solution!");
  }

  return newton_optimizer.getTwistCommand(x_robot, U_optimal[0], params_.time_discretization);
}

}  // namespace trajectory_handlers
}  // namespace nav2_frenet_ilqr_controller
