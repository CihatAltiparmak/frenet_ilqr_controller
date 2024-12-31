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
#include <frenet_trajectory_planner/conversion_adapters/base_adapter.hpp>
#include <frenet_trajectory_planner/type_definitions.hpp>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

class LineAdapter : public BaseAdapter
{
public:
  LineAdapter(const CartesianPoint & start_point, const CartesianPoint & final_point);
  CartesianState convertFrenet2Cartesian(const FrenetState & frenet_state);
  FrenetState convertCartesian2Frenet(const CartesianState & cartesian_state);

private:
  Vector2d t_frenet_;
  Vector2d x0_;
};

LineAdapter::LineAdapter(const CartesianPoint & start_point, const CartesianPoint & final_point)
: BaseAdapter()
{
  x0_ = start_point;

  auto line_vec = final_point - start_point;
  t_frenet_ = line_vec / line_vec.norm();

  arclength_ = line_vec.norm();
}

CartesianState LineAdapter::convertFrenet2Cartesian(const FrenetState & frenet_state)
{
  CartesianState cartesian_state = CartesianState::Zero();

  Matrix<double, 2, 2> T;
  T << t_frenet_[0], -t_frenet_[1],
    t_frenet_[1], t_frenet_[0];

  cartesian_state({0, 3}) = T * frenet_state({0, 3}) + x0_;
  cartesian_state({1, 4}) = T * frenet_state({1, 4});
  cartesian_state({2, 5}) = T * frenet_state({2, 5});
  cartesian_state[6] = std::atan2(t_frenet_[1], t_frenet_[0]) + std::atan2(
    frenet_state[4],
    frenet_state[1]);

  return cartesian_state;

}

FrenetState LineAdapter::convertCartesian2Frenet(const CartesianState & cartesian_state)
{
  Matrix<double, 6, 2> T;
  T << (cartesian_state[0] - x0_[0]), cartesian_state[3] - x0_[1],
    cartesian_state[1], cartesian_state[4],
    cartesian_state[2], cartesian_state[5],
    (cartesian_state[3] - x0_[1]), -(cartesian_state[0] - x0_[0]),
    cartesian_state[4], -cartesian_state[1],
    cartesian_state[5], -cartesian_state[2];

  FrenetState frenet_state = T * t_frenet_;

  return frenet_state;
}

}
