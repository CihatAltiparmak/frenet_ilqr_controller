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

#pragma once

#include "nav2_frenet_ilqr_controller/trajectory_handlers/trajectory_handler.hpp"

#include "nav2_core/controller.hpp"
#include "ilqr_trajectory_tracker/models/diff_robot_model.hpp"
#include "ilqr_trajectory_tracker/ilqr_optimizer.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace trajectory_handlers
{

using frenet_trajectory_planner::CartesianState;
using frenet_trajectory_planner::CartesianTrajectory;
using ilqr_trajectory_tracker::DiffDriveRobotModel;

class DiffDriveTrajectoryHandler : public TrajectoryHandler
{
public:
  explicit DiffDriveTrajectoryHandler(const Parameters & params);
  Vector3d processTrajectory(
    const CartesianState & c_state_robot,
    const CartesianTrajectory & c_trajectory_robot) override;
};

}  // namespace trajectory_handlers
}  // namespace nav2_frenet_ilqr_controller
