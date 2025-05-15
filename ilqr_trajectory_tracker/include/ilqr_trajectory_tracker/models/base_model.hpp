#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

template<typename StateT, typename InputT>
class Model
{
public:
  Model();
  virtual StateT applySystemDynamics(const StateT & x, const InputT & u, const double dt) = 0;
  virtual InputT applyLimits(const InputT & u) = 0;
  void setLimits(const Vector2d & input_limits_min, const Vector2d & input_limits_max);
  virtual MatrixXd getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt) = 0;
  virtual MatrixXd getControlMatrix(
    const StateT & x_eq, const InputT & u_eq,
    const double dt) = 0;
protected:
  InputT input_limits_min_;
  InputT input_limits_max_;
};

template<typename StateT, typename InputT>
Model<StateT, InputT>::Model()
{
  input_limits_min_ << -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity();
  input_limits_max_ << +std::numeric_limits<double>::infinity(), +std::numeric_limits<double>::infinity();
}
template<typename StateT, typename InputT>



void Model<StateT, InputT>::setLimits(
  const Vector2d & input_limits_min,
  const Vector2d & input_limits_max) {
  input_limits_min_ = input_limits_min;
  input_limits_max_ = input_limits_max;
}
}
