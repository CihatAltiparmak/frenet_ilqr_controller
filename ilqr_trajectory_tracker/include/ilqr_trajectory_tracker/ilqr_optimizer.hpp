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

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

class Optimizer
{
public:
  Optimizer() {}
  // virtual void forward_pass();
  // virtual void backward_pass();
  // virtual void optimize();
};

template<typename RobotModel>
class NewtonOptimizer : public Optimizer
{
public:
  NewtonOptimizer();
  std::vector<MatrixXd> backwardPass(
    const std::vector<typename RobotModel::StateT> & x_feasible,
    const std::vector<typename RobotModel::InputT> & u_feasible,
    const MatrixXd & Q, const MatrixXd & R, const double dt);
  std::tuple<MatrixXd, MatrixXd> solveDiscreteLQRProblem(
    const MatrixXd & A, const MatrixXd & B,
    const MatrixXd & Q, const MatrixXd & R,
    const MatrixXd & P);
  std::tuple<std::vector<typename RobotModel::StateT>,
    std::vector<typename RobotModel::InputT>> forwardPass(
    const typename RobotModel::StateT & x0,
    const std::vector<typename RobotModel::StateT> & x_feasible,
    const std::vector<typename RobotModel::InputT> & u_feasible,
    const std::vector<MatrixXd> & K_gains, const double dt, const double alpha);

  std::vector<typename RobotModel::InputT> optimize(
    const typename RobotModel::StateT & x0,
    const std::vector<typename RobotModel::StateT> & x_feasible, const Matrix3d & Q,
    const Matrix2d & R, const double dt);
  double cost(
    const std::vector<typename RobotModel::StateT> & x_tracked,
    const std::vector<typename RobotModel::StateT> & x_trajectory);

  void setIterationNumber(const size_t iteration_number);
  void setAlpha(const double alpha);

private:
  RobotModel robot_model_;
  double alpha_;
  size_t iteration_number_;
};

template<typename RobotModel>
NewtonOptimizer<RobotModel>::NewtonOptimizer()
: Optimizer()
{
}

template<typename RobotModel>
std::vector<MatrixXd> NewtonOptimizer<RobotModel>::backwardPass(
  const std::vector<typename RobotModel::StateT> & x_feasible,
  const std::vector<typename RobotModel::InputT> & u_feasible, const MatrixXd & Q,
  const MatrixXd & R, const double dt)
{

  auto state_dimension = Q.rows();
  auto input_dimension = R.rows();

  MatrixXd P_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
  P_tilda.topLeftCorner(state_dimension, state_dimension) = Q;

  std::vector<MatrixXd> K_gain(x_feasible.size(),
    MatrixXd::Zero(input_dimension, state_dimension + 1));

  for (auto i = std::distance(x_feasible.begin(), std::prev(x_feasible.end(), 2)); i >= 0; i--) {
    auto x_offset =
      robot_model_.applySystemDynamics(x_feasible.at(i), u_feasible[i], dt) - x_feasible[i + 1];

    MatrixXd A_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
    auto A = robot_model_.getStateMatrix(x_feasible[i], u_feasible[i], dt);
    A_tilda.topLeftCorner(state_dimension, state_dimension) = A;
    A_tilda.topRightCorner(state_dimension, 1) = x_offset;

    MatrixXd B_tilda = MatrixXd::Zero(state_dimension + 1, input_dimension);
    auto B = robot_model_.getControlMatrix(x_feasible[i], u_feasible[i], dt);
    B_tilda.topLeftCorner(state_dimension, input_dimension) = B;

    MatrixXd Q_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
    Q_tilda.topLeftCorner(state_dimension, state_dimension) = Q;

    MatrixXd R_tilda = R;

    std::tie(K_gain[i], P_tilda) = solveDiscreteLQRProblem(
      A_tilda, B_tilda, Q_tilda, R_tilda,
      P_tilda);
  }

  return K_gain;
}

template<typename RobotModel>
std::tuple<std::vector<typename RobotModel::StateT>,
  std::vector<typename RobotModel::InputT>> NewtonOptimizer<RobotModel>::forwardPass(
  const typename RobotModel::StateT & x0,
  const std::vector<typename RobotModel::StateT> & x_feasible,
  const std::vector<typename RobotModel::InputT> & u_feasible,
  const std::vector<MatrixXd> & K_gains, const double dt, const double alpha)
{
  auto trajectory_size = x_feasible.size();
  std::vector<typename RobotModel::StateT> x_tracked(trajectory_size);

  auto input_size = u_feasible.size();
  std::vector<typename RobotModel::InputT> u_applied(input_size);

  // assert trajectory_size > 0
  x_tracked[0] = x0;
  for (typename std::vector<typename RobotModel::StateT>::difference_type i = 0,
    max_difference = std::distance(x_feasible.begin(), std::prev(x_feasible.end(), 1));
    i < max_difference; ++i)
  {
    auto x_error = x_tracked[i] - x_feasible[i];
    Vector<double, 4> z_error;
    z_error << x_error, alpha;

    u_applied[i] = u_feasible[i] + K_gains[i] * z_error;
    x_tracked[i + 1] = robot_model_.applySystemDynamics(x_tracked[i], u_applied[i], dt);
  }

  return {x_tracked, u_applied};
}

template<typename RobotModel>
std::tuple<MatrixXd, MatrixXd> NewtonOptimizer<RobotModel>::solveDiscreteLQRProblem(
  const MatrixXd & A, const MatrixXd & B, const MatrixXd & Q, const MatrixXd & R,
  const MatrixXd & P)
{
  auto BTmP = B.transpose() * P;
  auto K = -(R + BTmP * B).completeOrthogonalDecomposition().pseudoInverse() * BTmP * A;

  auto ApBK = (A + B * K);
  auto P_new = Q + K.transpose() * R * K + ApBK.transpose() * P * ApBK;

  return {K, P_new};
}

template<typename RobotModel>
std::vector<typename RobotModel::InputT> NewtonOptimizer<RobotModel>::optimize(
  const typename RobotModel::StateT & x0,
  const std::vector<typename RobotModel::StateT> & x_trajectory, const Matrix3d & Q,
  const Matrix2d & R, const double dt)
{
  // assert trajectory_size > 0

  double alpha = alpha_;

  std::vector<typename RobotModel::StateT> x_best_trajectory;
  std::vector<typename RobotModel::InputT> u_best_trajectory;
  std::vector<typename RobotModel::InputT> u_optimized(std::distance(
      x_trajectory.begin(),
      std::prev(x_trajectory.end(), 1)),
    RobotModel::InputT::Zero());

  double best_trajectory_cost = std::numeric_limits<double>::infinity();
  double previous_best_trajectory_cost = best_trajectory_cost;
  for (size_t i = 0; i < iteration_number_; ++i) {
    auto K_gain_list = this->backwardPass(x_trajectory, u_optimized, Q, R, dt);
    auto [x_tracked, u_tracked] = this->forwardPass(x0,
      x_trajectory, u_optimized, K_gain_list, dt,
      alpha);
    u_optimized = u_tracked;

    double trajectory_cost = this->cost(x_tracked, x_trajectory);
    if (trajectory_cost < best_trajectory_cost) {
      previous_best_trajectory_cost = best_trajectory_cost;
      best_trajectory_cost = trajectory_cost;
      x_best_trajectory = x_tracked;
      u_best_trajectory = u_tracked;

      if (std::abs(previous_best_trajectory_cost - best_trajectory_cost) < 0.0001) {
        break;
      }

      alpha *= 0.7;
    } else {
      alpha /= 0.7;
    }
  }

  return u_best_trajectory;
}

template<typename RobotModel>
double NewtonOptimizer<RobotModel>::cost(
  const std::vector<typename RobotModel::StateT> & x_tracked,
  const std::vector<typename RobotModel::StateT> & x_trajectory)
{
  double trajectory_cost = 0;
  for (typename std::vector<typename RobotModel::StateT>::size_type i = 0; i < x_trajectory.size();
    ++i)
  {
    trajectory_cost += (x_tracked[i] - x_trajectory[i]).squaredNorm();
  }

  return trajectory_cost;
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::setIterationNumber(const size_t iteration_number)
{
  iteration_number_ = iteration_number;
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::setAlpha(const double alpha)
{
  alpha_ = alpha;
}

}
