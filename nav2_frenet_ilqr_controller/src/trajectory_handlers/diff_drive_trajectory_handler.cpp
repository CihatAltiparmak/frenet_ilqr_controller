// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_frenet_ilqr_controller/trajectory_handlers/diff_drive_trajectory_handler.hpp"

#include "nav2_core/controller_exceptions.hpp"
#include "ilqr_trajectory_tracker/models/diff_robot_model.hpp"
#include "ilqr_trajectory_tracker/ilqr_optimizer.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace trajectory_handlers
{

using frenet_trajectory_planner::CartesianState;
using frenet_trajectory_planner::CartesianTrajectory;
using ilqr_trajectory_tracker::DiffDriveRobotModel;

DiffDriveTrajectoryHandler::DiffDriveTrajectoryHandler(const Parameters & params)
: TrajectoryHandler(params)
{}

Vector3d DiffDriveTrajectoryHandler::processTrajectory(
  const CartesianState & c_state_robot,
  const CartesianTrajectory & c_trajectory_robot)
{
  if (c_trajectory_robot.empty()) {
    throw nav2_core::NoValidControl("There is no trajectory to be tracked!");
  }

  ilqr_trajectory_tracker::NewtonOptimizer<DiffDriveRobotModel> newton_optimizer;

  auto x_robot = DiffDriveRobotModel::fromFrenetCartesianState(c_state_robot);
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
    DiffDriveRobotModel::StateT x_stop = X_feasible.back();
    x_stop[3] = 0.0;
    for (size_t i = 0; i < state_number_for_stopping; ++i) {
      X_feasible.push_back(x_stop);
    }
  }

  newton_optimizer.setIterationNumber(params_.iteration_number);
  newton_optimizer.setInputConstraints(params_.input_limits_min, params_.input_limits_max);
  auto U_optimal = newton_optimizer.optimize(x_robot, X_feasible, params_.Q, params_.R,
      params_.time_discretization);

  if (U_optimal.empty()) {
    throw nav2_core::NoValidControl("Iterative LQR couldn't find any solution!");
  }

  return newton_optimizer.getTwistCommand(x_robot, U_optimal[0], params_.time_discretization);
}

}  // namespace trajectory_handlers
}  // namespace nav2_frenet_ilqr_controller
