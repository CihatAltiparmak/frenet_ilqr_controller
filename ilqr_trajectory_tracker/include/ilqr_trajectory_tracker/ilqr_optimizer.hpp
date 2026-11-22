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
#include <cmath>
#include <memory>
#include <tuple>
#include <algorithm>
#include <limits>
#include <vector>
#include "frenet_trajectory_planner/type_definitions.hpp"
#include <iostream>

using namespace Eigen;  // NOLINT

namespace ilqr_trajectory_tracker
{

class Optimizer
{
public:
  Optimizer() {}
  // virtual void forward_pass();
  // virtual void backward_pass();
  // virtual void optimize();
  // Vecttor2d getTwistCommand();
};

template<typename RobotModel>
class NewtonOptimizer : public Optimizer
{
public:
  static const size_t StateDim {RobotModel::StateDim};
  static const size_t InputDim {RobotModel::InputDim};
  using StateT = typename RobotModel::StateT;
  using InputT = typename RobotModel::InputT;
  using StateMatrixT = typename RobotModel::StateMatrixT;
  using ControlMatrixT = typename RobotModel::ControlMatrixT;

  template<typename ... RobotModelParams>
  explicit NewtonOptimizer(const RobotModelParams ... model_params);

  std::vector<std::tuple<MatrixXd, VectorXd>> backwardPass(
    const std::vector<StateT> & x_feasible,
    const std::vector<InputT> & u_feasible,
    const Matrix<double, StateDim, StateDim> & Q,
    const Matrix<double, InputDim, InputDim> & R,
    const double dt)
  {
    MatrixXd P_tilda = MatrixXd::Identity(StateDim + 1, StateDim + 1);
    P_tilda.topLeftCorner(StateDim, StateDim) = Q;

    VectorXd p_tilda = VectorXd::Ones(StateDim + 1);

    std::vector<std::tuple<MatrixXd, VectorXd>> K_gain(x_feasible.size() - 1,
      std::make_pair(MatrixXd::Zero(InputDim, StateDim + 1), VectorXd::Zero(StateDim + 1)));

    // assert trajectory_size > 1
    for (size_t i = x_feasible.size() - 1; i > 0; --i) {
      auto x_offset =
        robot_model_->applySystemDynamics(x_feasible[i - 1], u_feasible[i - 1], dt) - x_feasible[i];

      MatrixXd A_tilda = MatrixXd::Identity(StateDim + 1, StateDim + 1);
      StateMatrixT A = robot_model_->getStateMatrix(x_feasible[i - 1], u_feasible[i - 1], dt);
      A_tilda.topLeftCorner(StateDim, StateDim) = A;
      A_tilda.topRightCorner(StateDim, 1) = x_offset;

      MatrixXd B_tilda = MatrixXd::Zero(StateDim + 1, InputDim);
      ControlMatrixT B = robot_model_->getControlMatrix(x_feasible[i - 1], u_feasible[i - 1], dt);
      B_tilda.topLeftCorner(StateDim, InputDim) = B;

      MatrixXd Q_tilda = MatrixXd::Identity(StateDim + 1, StateDim + 1);
      Q_tilda.topLeftCorner(StateDim, StateDim) = Q;

      MatrixXd R_tilda = R;

      std::tie(K_gain[i - 1], P_tilda, p_tilda) = solveDiscreteLQRProblem(
        A_tilda, B_tilda, Q_tilda, R_tilda,
        P_tilda, p_tilda, x_feasible[i - 1], u_feasible[i - 1]);
    }

    return K_gain;
  }

  std::tuple<std::tuple<MatrixXd, VectorXd>, MatrixXd, VectorXd> solveDiscreteLQRProblem(
    const MatrixXd & A, const MatrixXd & B,
    const MatrixXd & Q, const MatrixXd & R,
    const MatrixXd & P, const VectorXd & p,
    const VectorXd x,
    const VectorXd u)
  {
    Vector<double, InputDim> curvature_cost = Vector<double, InputDim>::Zero();
    double cost_derivative = 1.0 * ((2 * u[1]) / (x[3] * x[3]));
    if (std::isfinite(cost_derivative)) {
      curvature_cost[InputDim - 1] = cost_derivative;
    }

    auto BTmP = B.transpose() * P;
    auto K = -(R + BTmP * B).completeOrthogonalDecomposition().pseudoInverse() * BTmP * A;
    auto k = -(R + BTmP * B).completeOrthogonalDecomposition().pseudoInverse() * B.transpose() * p - curvature_cost;

    auto ApBK = (A + B * K);
    auto P_new = Q + K.transpose() * R * K + ApBK.transpose() * P * ApBK;
    auto p_new = K.transpose() * R * k + ApBK.transpose() * (P * B * k + p);

    return {std::make_pair(K, k), P_new, p_new};
  }

  std::tuple<std::vector<StateT>,
    std::vector<InputT>> forwardPass(
    const StateT & x0,
    const std::vector<StateT> & x_feasible,
    const std::vector<InputT> & u_feasible,
    const std::vector<std::tuple<MatrixXd, VectorXd>> & K_gains, const double dt)
  {
    auto trajectory_size = x_feasible.size();
    std::vector<StateT> x_tracked(trajectory_size);

    auto input_size = u_feasible.size();
    std::vector<InputT> u_applied(input_size);

    // assert trajectory_size > 0
    x_tracked[0] = x0;
    for (size_t i = 0; i < x_feasible.size() - 1; ++i) {
      auto x_error = x_tracked[i] - x_feasible[i];
      VectorXd z_error(StateDim + 1);
      z_error << x_error, 1.0;

      auto [K, k] = K_gains[i];
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wnull-dereference"
      u_applied[i] = u_feasible[i] + K * z_error + k;
      #pragma GCC diagnostic pop
      u_applied[i] = robot_model_->applyLimits(u_applied[i]);
      x_tracked[i + 1] = robot_model_->applySystemDynamics(x_tracked[i], u_applied[i], dt);
    }

    return {x_tracked, u_applied};
  }

  std::vector<InputT> optimize(
    const StateT & x0,
    const std::vector<StateT> & x_trajectory,
    const Matrix<double, StateDim, StateDim> & Q,
    const Matrix<double, InputDim, InputDim> & R,
    const double dt)
  {
    // assert trajectory_size > 0
    std::vector<StateT> x_best_trajectory;
    std::vector<InputT> u_best_trajectory;
    std::vector<InputT> u_optimized(x_trajectory.size() - 1, InputT::Zero());

    // for (size_t i = 1; i < u_optimized.size(); ++i) {
    //   u_optimized[i][0] = (x_trajectory[i][3] - x_trajectory[i - 1][3]) / dt;
    //   u_optimized[i][1] = (x_trajectory[i][2] - x_trajectory[i - 1][2]) / dt;
    // }
    // u_optimized[0][0] = (x_trajectory[0][3] - x0[3]) / dt;
    // u_optimized[0][1] = (x_trajectory[0][2] - x0[2]) / dt;
    // std::cout << "PREDICTED: " << u_optimized[0][0] << ", " << u_optimized[0][1] << std::endl;

    double best_trajectory_cost = std::numeric_limits<double>::infinity();
    double previous_best_trajectory_cost = best_trajectory_cost;

    std::vector<StateT> x_traj_last;
    std::vector<InputT> u_traj_last;
    for (size_t i = 0; i < iteration_number_; ++i) {
      auto K_gain_list = this->backwardPass(x_trajectory, u_optimized, Q, R, dt);
      auto [x_tracked, u_tracked] = this->forwardPass(x0,
        x_trajectory, u_optimized, K_gain_list, dt);
      u_optimized = u_tracked;

      x_traj_last = x_tracked;
      u_traj_last = u_tracked;

      double trajectory_cost = this->cost(x_tracked, x_trajectory);
      if (trajectory_cost < best_trajectory_cost) {
        previous_best_trajectory_cost = best_trajectory_cost;
        best_trajectory_cost = trajectory_cost;
        x_best_trajectory = x_tracked;
        u_best_trajectory = u_tracked;

        // if (std::abs(previous_best_trajectory_cost - best_trajectory_cost) < 0.0001) {
        //   break;
        // }
      }
    }

    // if (u_best_trajectory.empty()) {
    //   for (size_t i = 0; i < x_trajectory.size(); ++i) {
    //     std::cout 
    //       << x_trajectory[i][0] << ", "
    //       << x_trajectory[i][1] << ", "
    //       << x_trajectory[i][2] << ", "
    //       << x_trajectory[i][3] << std::endl;
    //     std::cout 
    //       << x_traj_last[i][0] << ", "
    //       << x_traj_last[i][1] << ", "
    //       << x_traj_last[i][2] << ", "
    //       << x_traj_last[i][3] << std::endl;
    //     std::cout << "**************" << std::endl;
    //   }
    //   std::cout << "##############" << std::endl; 
    //   for (size_t i = 0; i < u_traj_last.size(); ++i) {
    //     std::cout 
    //       << u_traj_last[i][0] << ", "
    //       << u_traj_last[i][1] << std::endl;
    //   }
    //   std::cout << "--------------" << std::endl;
    //    std::cout.flush();
    // }

    // std::cout << "OPTIMIZED: " << u_best_trajectory[0][0] << ", " << u_best_trajectory[0][1] << std::endl;

    return u_best_trajectory;
  }

  Vector3d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt
  )
  {
    return robot_model_->getTwistCommand(x_initial, u, dt);
  }

  double cost(
    const std::vector<StateT> & x_tracked,
    const std::vector<StateT> & x_trajectory)
  {
    double trajectory_cost = 0;
    for (size_t i = 0; i < x_trajectory.size();
      ++i)
    {
      trajectory_cost += (x_tracked[i] - x_trajectory[i]).squaredNorm();
    }

    return trajectory_cost;
  }

  std::vector<StateT> fromFrenetCartesianTrajectory(
    const frenet_trajectory_planner::CartesianTrajectory & c_trajectory)
  {
    std::vector<StateT> converted_trajectory;
    for (auto c_state : c_trajectory) {
      StateT converted_state = RobotModel::fromFrenetCartesianState(c_state);
      converted_trajectory.push_back(converted_state);
    }

    return converted_trajectory;
  }

  void setIterationNumber(const size_t iteration_number);

  void setInputConstraints(
    InputT input_limits_min,
    InputT input_limits_max);

private:
  std::unique_ptr<RobotModel> robot_model_;
  size_t iteration_number_;
};

template<typename RobotModel>
template<typename ... RobotModelParams>
NewtonOptimizer<RobotModel>::NewtonOptimizer(const RobotModelParams ... model_params)
: Optimizer()
{
  robot_model_ = std::make_unique<RobotModel>(model_params ...);
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::setIterationNumber(const size_t iteration_number)
{
  iteration_number_ = iteration_number;
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::setInputConstraints(
  InputT input_limits_min,
  InputT input_limits_max)
{
  robot_model_->setLimits(input_limits_min, input_limits_max);
}

}  // namespace ilqr_trajectory_tracker
