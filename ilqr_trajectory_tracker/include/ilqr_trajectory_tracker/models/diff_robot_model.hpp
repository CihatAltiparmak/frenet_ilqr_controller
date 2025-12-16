#pragma once

#include <ilqr_trajectory_tracker/models/base_model.hpp>

#include <Eigen/Dense>
#include <cmath>
#include "frenet_trajectory_planner/type_definitions.hpp"

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

using DiffDriveRobotModelState = Vector4d;
using DiffDriveRobotModelInput = Vector2d;

class DiffDriveRobotModel : public Model<4, 2>
{
public:
  DiffDriveRobotModel();
  StateT applySystemDynamics(const StateT & x, const InputT & u, const double dt) override;
  InputT applyLimits(const InputT & u) override;
  StateMatrixT getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  ControlMatrixT getControlMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  Vector2d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt);
  
  static StateT fromFrenetCartesianState(const frenet_trajectory_planner::CartesianState & c_state);
};

}
