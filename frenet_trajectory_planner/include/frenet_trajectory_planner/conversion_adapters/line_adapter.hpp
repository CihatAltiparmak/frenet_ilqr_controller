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
