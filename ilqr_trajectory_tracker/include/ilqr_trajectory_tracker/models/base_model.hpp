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
  virtual MatrixXd getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt) = 0;
  virtual MatrixXd getControlMatrix(
    const StateT & x_eq, const InputT & u_eq,
    const double dt) = 0;
};

template<typename StateT, typename InputT>
Model<StateT, InputT>::Model()
{
}

}
