#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <tuple>
#include <algorithm>
<<<<<<< HEAD
=======
#include <limits>
#include "frenet_trajectory_planner/type_definitions.hpp"
>>>>>>> aa7ebd1 (Improve templates in ILQR module for better supporting additional models (#51))

#include <limits>
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
  // Vecttor2d getTwistCommand();
};

template<typename RobotModel>
class NewtonOptimizer : public Optimizer
{
public:
  static const size_t StateDim {RobotModel::StateDim};
  static const size_t InputDim {RobotModel::InputDim};
  using StateT         = typename RobotModel::StateT;
  using InputT         = typename RobotModel::InputT;
  using StateMatrixT   = typename RobotModel::StateMatrixT;
  using ControlMatrixT = typename RobotModel::ControlMatrixT;

  template<typename ... RobotModelParams>
  NewtonOptimizer(const RobotModelParams ... model_params);
  std::vector<MatrixXd> backwardPass(
<<<<<<< HEAD
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
=======
    const std::vector<StateT> & x_feasible,
    const std::vector<InputT> & u_feasible,
    const Matrix<double, StateDim, StateDim> & Q, 
    const Matrix<double, InputDim, InputDim> & R,
    const double dt) {

    MatrixXd P_tilda = MatrixXd::Identity(StateDim + 1, StateDim + 1);
    P_tilda.topLeftCorner(StateDim, StateDim) = Q;
      
    std::vector<MatrixXd> K_gain(x_feasible.size() - 1,
      MatrixXd::Zero(InputDim, StateDim + 1));

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

      std::tie(K_gain[i - 1], P_tilda) = solveDiscreteLQRProblem(
        A_tilda, B_tilda, Q_tilda, R_tilda,
        P_tilda);
    }

    return K_gain;
  }

  std::tuple<MatrixXd, MatrixXd> solveDiscreteLQRProblem(
    const MatrixXd & A, const MatrixXd & B,
    const MatrixXd & Q, const MatrixXd & R,
    const MatrixXd & P) {

    auto BTmP = B.transpose() * P;
    auto K = -(R + BTmP * B).completeOrthogonalDecomposition().pseudoInverse() * BTmP * A;
>>>>>>> aa7ebd1 (Improve templates in ILQR module for better supporting additional models (#51))

    auto ApBK = (A + B * K);
    auto P_new = Q + K.transpose() * R * K + ApBK.transpose() * P * ApBK;

    return {K, P_new};
  }

  std::tuple<std::vector<StateT>,
    std::vector<InputT>> forwardPass(
    const StateT & x0,
    const std::vector<StateT> & x_feasible,
    const std::vector<InputT> & u_feasible,
    const std::vector<MatrixXd> & K_gains, const double dt, const double alpha) {

    auto trajectory_size = x_feasible.size();
    std::vector<StateT> x_tracked(trajectory_size);

    auto input_size = u_feasible.size();
    std::vector<InputT> u_applied(input_size);

    // assert trajectory_size > 0
    x_tracked[0] = x0;
    for (size_t i = 0; i < x_feasible.size() - 1; ++i)
    {
      auto x_error = x_tracked[i] - x_feasible[i];
      VectorXd z_error(StateDim + 1);
      z_error << x_error, alpha;

      u_applied[i] = u_feasible[i] + K_gains[i] * z_error;
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
    const double dt) {
    // assert trajectory_size > 0

    double alpha = alpha_;

    std::vector<StateT> x_best_trajectory;
    std::vector<InputT> u_best_trajectory;
    std::vector<InputT> u_optimized(x_trajectory.size() - 1, InputT::Zero());

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

        alpha *= 0.5;
      } else {
        alpha /= 0.5;
      }
    }

    return u_best_trajectory;
  }

  Vector2d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt
  ) {
    return robot_model_->getTwistCommand(x_initial, u, dt);
  }

  double cost(
    const std::vector<StateT> & x_tracked,
    const std::vector<StateT> & x_trajectory) {
    double trajectory_cost = 0;
    for (size_t i = 0; i < x_trajectory.size();
      ++i)
    {
      trajectory_cost += (x_tracked[i] - x_trajectory[i]).squaredNorm();
    }

    return trajectory_cost;
  }

  std::vector<StateT> fromFrenetCartesianTrajectory(const frenet_trajectory_planner::CartesianTrajectory & c_trajectory) {
    std::vector<StateT> converted_trajectory;
    for (auto c_state : c_trajectory) {
      StateT converted_state = RobotModel::fromFrenetCartesianState(c_state);
      converted_trajectory.push_back(converted_state);
    }

    return converted_trajectory;
  }

  void setIterationNumber(const size_t iteration_number);
  void setAlpha(const double alpha);

  void setInputConstraints(
<<<<<<< HEAD
    typename RobotModel::InputT input_limits_min,
    typename RobotModel::InputT input_limits_max);
=======
    InputT input_limits_min,
    InputT input_limits_max);

>>>>>>> aa7ebd1 (Improve templates in ILQR module for better supporting additional models (#51))
private:
  std::unique_ptr<RobotModel> robot_model_;
  double alpha_;
  size_t iteration_number_;
};

template<typename RobotModel>
template<typename ... RobotModelParams>
NewtonOptimizer<RobotModel>::NewtonOptimizer(const RobotModelParams ... model_params)
: Optimizer()
{
  robot_model_ = std::make_unique<RobotModel>(model_params...);
}

template<typename RobotModel>
<<<<<<< HEAD
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
      robot_model_->applySystemDynamics(x_feasible.at(i), u_feasible[i], dt) - x_feasible[i + 1];
    MatrixXd A_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
    auto A = robot_model_->getStateMatrix(x_feasible[i], u_feasible[i], dt);
    A_tilda.topLeftCorner(state_dimension, state_dimension) = A;
    A_tilda.topRightCorner(state_dimension, 1) = x_offset;

    MatrixXd B_tilda = MatrixXd::Zero(state_dimension + 1, input_dimension);
    auto B = robot_model_->getControlMatrix(x_feasible[i], u_feasible[i], dt);
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
  for (size_t i = 0; i < x_feasible.size() - 1; ++i)
  {
    auto x_error = x_tracked[i] - x_feasible[i];
    Vector<double, 5> z_error;
    z_error << x_error, alpha;

    u_applied[i] = u_feasible[i] + K_gains[i] * z_error;
    u_applied[i] = robot_model_->applyLimits(u_applied[i]);
    x_tracked[i + 1] = robot_model_->applySystemDynamics(x_tracked[i], u_applied[i], dt);

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
  const std::vector<typename RobotModel::StateT> & x_trajectory, const Matrix4d & Q,
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

      alpha *= 0.5;
    } else {
      alpha /= 0.5;
    }
  }


  return u_best_trajectory;
}

template<typename RobotModel>
Vector2d NewtonOptimizer<RobotModel>::getTwistCommand(
  const typename RobotModel::StateT & x_initial,
  const typename RobotModel::InputT & u,
  const double dt)
{
  return robot_model_->getTwistCommand(x_initial, u, dt);
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
=======
>>>>>>> aa7ebd1 (Improve templates in ILQR module for better supporting additional models (#51))
void NewtonOptimizer<RobotModel>::setIterationNumber(const size_t iteration_number)
{
  iteration_number_ = iteration_number;
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::setAlpha(const double alpha)
{
  alpha_ = alpha;
}


template<typename RobotModel>


void NewtonOptimizer<RobotModel>::setInputConstraints(
  InputT input_limits_min,
  InputT input_limits_max)
{
  robot_model_->setLimits(input_limits_min, input_limits_max);
}
}
