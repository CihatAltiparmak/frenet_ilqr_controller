#include <ilqr_trajectory_tracker/models/diff_robot_model.hpp>
#include <ilqr_trajectory_tracker/ilqr_optimizer.hpp>
#include <limits>
#include <iostream>
#include <chrono>

#define _USE_MATH_DEFINES

int main(int argc, char ** argv)
{
  using ilqr_trajectory_tracker::DiffDriveRobotModel;
  using ilqr_trajectory_tracker::DiffDriveRobotModelState;
  using ilqr_trajectory_tracker::DiffDriveRobotModelInput;

  DiffDriveRobotModel diff_drive_robot_model;

  int trajectory_size = 57;
  std::vector<DiffDriveRobotModelState> x_feasible(trajectory_size,
    DiffDriveRobotModelState::Zero());
  x_feasible[0] << 5, 0, M_PI / 2;

  DiffDriveRobotModelInput u_applied;
  u_applied << 5 * M_PI, M_PI;
  std::vector<DiffDriveRobotModelInput> u_ground_truth(trajectory_size - 1, u_applied);

  double dt = 0.05;
  for (int i = 0; i < trajectory_size - 1; i++) {
    x_feasible[i + 1] = diff_drive_robot_model.apply_system_dynamics(
      x_feasible[i], u_ground_truth[i],
      dt);
  }

  Matrix3d Q = Matrix3d::Identity() * 10;
  Matrix2d R = Matrix2d::Identity() * 0.1;
  double alpha = 1;

  ilqr_trajectory_tracker::NewtonOptimizer<DiffDriveRobotModel> newton_optimizer;
  newton_optimizer.setIterationNumber(10);
  newton_optimizer.setAlpha(alpha);

  const auto start{std::chrono::steady_clock::now()};
  auto u_optimal = newton_optimizer.optimize(x_feasible, Q, R, dt);
  const auto end{std::chrono::steady_clock::now()};

  const std::chrono::duration<double> elapsed_seconds{end - start};
  std::cout << elapsed_seconds.count() << "s\n";


  return 0;
}
