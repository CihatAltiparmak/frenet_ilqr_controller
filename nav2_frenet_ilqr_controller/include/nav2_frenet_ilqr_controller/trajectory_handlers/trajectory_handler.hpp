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

#include <Eigen/Dense>
#include "frenet_trajectory_planner/type_definitions.hpp"
#include "nav2_frenet_ilqr_controller/parameter_handler.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace trajectory_handlers
{

using frenet_trajectory_planner::CartesianState;
using frenet_trajectory_planner::CartesianTrajectory;

class TrajectoryHandler {
public:
  explicit TrajectoryHandler(const Parameters & params)
  {
    params_ = params;
  }

  virtual ~TrajectoryHandler() = default;
  virtual Vector3d processTrajectory(
    const CartesianState & c_state_robot,
    const CartesianTrajectory & c_trajectory_robot) = 0;

protected:
  Parameters params_;
};
}  // namespace trajectory_handlers
}  // namespace nav2_frenet_ilqr_controller
