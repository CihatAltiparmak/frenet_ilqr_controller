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

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

template<size_t _StateDim, size_t _InputDim>
class Model
{
public:
  static const size_t StateDim = _StateDim;
  static const size_t InputDim = _InputDim;
  using StateT = Vector<double, StateDim>;
  using StateMatrixT = Matrix<double, StateDim, StateDim>;
  using InputT = Vector<double, InputDim>;
  using ControlMatrixT = Matrix<double, StateDim, InputDim>;

  Model();
  virtual StateT applySystemDynamics(const StateT & x, const InputT & u, const double dt) = 0;

  virtual InputT applyLimits(const InputT & u) = 0;

  void setLimits(const InputT & input_limits_min, const InputT & input_limits_max);

  virtual StateMatrixT getStateMatrix(const StateT & x_eq, const InputT & u_eq, const double dt) = 0;

  virtual ControlMatrixT getControlMatrix(
    const StateT & x_eq, const InputT & u_eq,
    const double dt) = 0;

  virtual Vector2d getTwistCommand(
    const StateT & x_initial,
    const InputT & u,
    const double dt) = 0;

  static size_t getStateDim();
  static size_t getInputDim();

protected:
    InputT input_limits_min_;
    InputT input_limits_max_;
};

template<size_t _StateDim, size_t _InputDim>
Model<_StateDim, _InputDim>::Model()
{
  input_limits_min_.fill(-std::numeric_limits<double>::infinity());
  input_limits_max_.fill(+std::numeric_limits<double>::infinity());
}

template<size_t _StateDim, size_t _InputDim>
void Model<_StateDim, _InputDim>::setLimits(
  const InputT & input_limits_min,
  const InputT & input_limits_max) {

  input_limits_min_ = input_limits_min;
  input_limits_max_ = input_limits_max;
}

template<size_t _StateDim, size_t _InputDim>
size_t Model<_StateDim, _InputDim>::getStateDim() {
  return _StateDim;
}

template<size_t _StateDim, size_t _InputDim>
size_t Model<_StateDim, _InputDim>::getInputDim() {
  return InputDim;
}

}
