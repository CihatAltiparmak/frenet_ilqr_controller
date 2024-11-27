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

Vector3d DiffDriveRobotModel::applySystemDynamics(
  const Vector3d & x, const Vector2d & u,
  const double dt)
{
  Vector3d x_final;
  x_final <<
    x[0] + u[0] * std::cos(x[2] + u[1] * dt) * dt,
    x[1] + u[0] * std::sin(x[2] + u[1] * dt) * dt,
    x[2] + u[1] * dt;

  return x_final;
}

MatrixXd DiffDriveRobotModel::getStateMatrix(
  const Vector3d & x_eq, const Vector2d & u_eq,
  const double dt)
{
  Matrix3d state_matrix;
  state_matrix << 1, 0, -u_eq[0] * std::sin(x_eq[2]) * dt,
    0, 1, u_eq[0] * std::cos(x_eq[2]) * dt,
    0, 0, 1;

  return state_matrix;
}

MatrixXd DiffDriveRobotModel::getControlMatrix(
  const Vector3d & x_eq, const Vector2d & /*u_eq*/,
  const double dt)
{
  Matrix<double, 3, 2> control_matrix;
  control_matrix << std::cos(x_eq[2]) * dt, 0,
    std::sin(x_eq[2]) * dt, 0,
    0, dt;

  return control_matrix;
}

}
