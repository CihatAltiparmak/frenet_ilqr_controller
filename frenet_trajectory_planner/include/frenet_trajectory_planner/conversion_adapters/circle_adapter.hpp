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

class CircleAdapter : public BaseAdapter
{
public:
  CircleAdapter(const CartesianPoint & start_point, const CartesianPoint & final_point);
  CircleAdapter(
    const CartesianPoint & start_point, const CartesianPoint & final_point,
    const Vector2d & c, const double r, const Vector2d & x, const Vector2d & y,
    const double alpha);
  CircleAdapter(
    const CartesianPoint & start_point, const CartesianPoint & final_point,
    const Vector2d & c, const double r, const Vector2d & x, const Vector2d & y,
    const double alpha, const int sign_indicactor);
  CartesianState convertFrenet2Cartesian(const FrenetState & frenet_state);
  FrenetState convertCartesian2Frenet(const CartesianState & cartesian_state);

private:
  Vector2d c_;
  double r_;
  Vector2d x_;
  Vector2d y_;
  double alpha_;
  int sign_indicactor_;

};

CircleAdapter::CircleAdapter(
  const CartesianPoint & /*start_point*/,
  const CartesianPoint & /*final_point*/)
: BaseAdapter()
{
}

CircleAdapter::CircleAdapter(
  const CartesianPoint & /*start_point*/, const CartesianPoint & /*final_point*/,
  const Vector2d & c, const double r, const Vector2d & x,
  const Vector2d & y, const double alpha)
: BaseAdapter(), c_(c), r_(r), x_(x), y_(y), alpha_(alpha), sign_indicactor_(1)
{

}

CircleAdapter::CircleAdapter(
  const CartesianPoint & /*start_point*/, const CartesianPoint & /*final_point*/,
  const Vector2d & c, const double r, const Vector2d & x,
  const Vector2d & y, const double alpha, const int sign_indicactor)
: BaseAdapter(), c_(c), r_(r), x_(x), y_(y), alpha_(alpha), sign_indicactor_(sign_indicactor)
{
  arclength_ = alpha_ * r_;
}

CartesianState CircleAdapter::convertFrenet2Cartesian(const FrenetState & frenet_state)
{
  CartesianState cartesian_state = CartesianState::Zero();

  const double cos_sDr = std::cos(frenet_state[0] / r_);
  const double sin_sDr = std::sin(frenet_state[0] / r_);
  const double rPd = r_ + sign_indicactor_ * frenet_state[3];

  const Vector2d x_cosPy_sin = x_ * cos_sDr + y_ * sin_sDr;
  const Vector2d y_cosMx_sin = y_ * cos_sDr - x_ * sin_sDr;

  cartesian_state({0, 3}) =
    c_ + rPd * x_cosPy_sin;
  cartesian_state({1, 4}) =
    (frenet_state[1] * rPd / r_) * y_cosMx_sin +
    (sign_indicactor_ * frenet_state[4]) * x_cosPy_sin;
  cartesian_state({2, 5}) =
    ((frenet_state[2] * rPd + sign_indicactor_ * 2 * frenet_state[1] * frenet_state[4]) / r_) *
    y_cosMx_sin +
    (-std::pow(
      frenet_state[1],
      2) * rPd / std::pow(r_, 2) + sign_indicactor_ * frenet_state[5]) * x_cosPy_sin;

  // TODO (CihatAltiparmak) : Implement in a correct way. This solution may not be elegant. 
  cartesian_state[6] = std::atan2(cartesian_state[4], cartesian_state[1]);
  // cartesian_state[6] = std::atan2(t_frenet_[1], t_frenet_[0]) + std::atan2(
  //   frenet_state[4],
  //   frenet_state[1]);

  return cartesian_state;

}

FrenetState CircleAdapter::convertCartesian2Frenet(const CartesianState & cartesian_state)
{
  FrenetState frenet_state = FrenetState::Zero();
  const Vector2d x_c = cartesian_state({0, 3});

  Vector2d xcMc = x_c - c_;
  double xcMc_norm = xcMc.norm();
  double xcMc_norm_squared = xcMc.squaredNorm();
  Vector2d _part1 = xcMc / xcMc_norm;
  Vector2d _part2;
  _part2 << -xcMc[1], xcMc[0];

  frenet_state[0] = r_ * std::atan2(_part1.dot(y_), _part1.dot(x_));

  const double cos_sDr = std::cos(frenet_state[0] / r_);
  const double sin_sDr = std::sin(frenet_state[0] / r_);

  const Vector2d y_cosMx_sin = y_ * cos_sDr - x_ * sin_sDr;
  const Vector2d x_cosPy_sin = x_ * cos_sDr + y_ * sin_sDr;

  Vector2d _part3 =
    (cartesian_state(
      {2,
        5}) * xcMc_norm_squared -
    xcMc.dot(
      cartesian_state(
        {1,
          4})) * cartesian_state({1, 4})) / (xcMc_norm_squared * xcMc_norm);

  frenet_state[1] = (r_ / xcMc.norm()) * (y_cosMx_sin.dot(cartesian_state({1, 4})));
  frenet_state[2] = frenet_state[1] *
    x_cosPy_sin.dot(cartesian_state({1, 4})) / xcMc_norm + r_ * y_cosMx_sin.dot(_part3);


  frenet_state[3] = xcMc.norm() - r_;
  frenet_state[4] = xcMc.dot(cartesian_state({1, 4})) / xcMc_norm;

  double _part4 =
    (xcMc.dot(
      cartesian_state(
        {2,
          5})) +
    cartesian_state({1, 4}).squaredNorm()) * xcMc_norm - std::pow(
    xcMc.dot(
      cartesian_state(
        {1,
          4})),
    2.0) / xcMc_norm;
  frenet_state[5] = _part4 / xcMc_norm_squared;
  return frenet_state;
}

}
