#include <ilqr_trajectory_tracker/models/diff_robot_model.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

DiffDriveRobotModel::DiffDriveRobotModel()
: Model<DiffDriveRobotModelState, DiffDriveRobotModelInput>()
{

}

DiffDriveRobotModelState DiffDriveRobotModel::applySystemDynamics(
  const DiffDriveRobotModelState & x, const DiffDriveRobotModelInput & u,
  const double dt)
{
  DiffDriveRobotModelState x_final;
  x_final <<
    x[0] + x[3] * std::cos(x[2]) * dt,
    x[1] + x[3] * std::sin(x[2]) * dt,
    x[2] + u[1] * dt,
    x[3] + u[0] * dt;

  return x_final;
}

DiffDriveRobotModelInput DiffDriveRobotModel::applyLimits(const DiffDriveRobotModelInput & u) {
  return u.cwiseMin(input_limits_max_).cwiseMax(input_limits_min_);
}

MatrixXd DiffDriveRobotModel::getStateMatrix(
  const DiffDriveRobotModelState & x_eq, const DiffDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix4d state_matrix;
  state_matrix << 
    1, 0, -x_eq[3] * std::sin(x_eq[2]) * dt, std::cos(x_eq[2]) * dt,
    0, 1, +x_eq[3] * std::cos(x_eq[2]) * dt, std::sin(x_eq[2]) * dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  return state_matrix;
}

MatrixXd DiffDriveRobotModel::getControlMatrix(
  const DiffDriveRobotModelState & x_eq, const DiffDriveRobotModelInput & u_eq,
  const double dt)
{
  Matrix<double, 4, 2> control_matrix;
  control_matrix <<
    0, 0,
    0, 0,
    0, dt,
    dt, 0;

  return control_matrix;
}

Vector2d DiffDriveRobotModel::getTwistCommand(
  const DiffDriveRobotModelState & x_initial,
  const DiffDriveRobotModelInput & u,
  const double dt
)
{
  Vector2d twist;
  // velocity_new = velocity_robot + acceleration * dt
  twist[0] = x_initial[3] + u[0] * dt;

  twist[1] = u[1];
  return twist;
}

}
