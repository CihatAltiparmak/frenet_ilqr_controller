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
