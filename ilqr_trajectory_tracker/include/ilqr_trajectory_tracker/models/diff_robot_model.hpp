#pragma once

#include <ilqr_trajectory_tracker/models/base_model.hpp>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

using DiffDriveRobotModelState = Vector4d;
using DiffDriveRobotModelInput = Vector2d;

class DiffDriveRobotModel : public Model<DiffDriveRobotModelState, DiffDriveRobotModelInput>
{
public:
  using StateT = DiffDriveRobotModelState;
  using InputT = DiffDriveRobotModelInput;
  DiffDriveRobotModel();
  DiffDriveRobotModelState applySystemDynamics(const StateT & x, const InputT & u, const double dt) override;
  InputT applyLimits(const InputT & u) override;
  MatrixXd getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  MatrixXd getControlMatrix(const StateT & x_eq, const InputT & u_eq, const double dt);
  Vector2d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt);
};

}
