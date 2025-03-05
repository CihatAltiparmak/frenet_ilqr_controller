// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Tufan Kumbasar, Istanbul Technical University Artificial Intelligence and Intelligent Systems (AI2S) Laboratory
// Copyright (C) 2024 Prof. Behçet Uğur Töreyin
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
    x_feasible[i + 1] = diff_drive_robot_model.applySystemDynamics(
      x_feasible[i], u_ground_truth[i],
      dt);
  }

  Matrix3d Q = Matrix3d::Identity() * 10;
  Matrix2d R = Matrix2d::Identity() * 0.1;

  ilqr_trajectory_tracker::NewtonOptimizer<DiffDriveRobotModel> newton_optimizer;
  newton_optimizer.setIterationNumber(10);

  const auto start{std::chrono::steady_clock::now()};
  // TODO: pass parameter of actual robot pose instead of 
  // passing the first state of feasible trajectory
  auto u_optimal = newton_optimizer.optimize(x_feasible[0], x_feasible, Q, R, dt);
  const auto end{std::chrono::steady_clock::now()};

  const std::chrono::duration<double> elapsed_seconds{end - start};
  std::cout << elapsed_seconds.count() << "s\n";


  return 0;
}
