#pragma once

#include <frenet_trajectory_planner/costs/base_cost.hpp>
#include <cmath>

namespace frenet_trajectory_planner
{
namespace costs
{

class LongtitutalVelocityCost : public Cost
{
public:
  LongtitutalVelocityCost(const double & K_longtitutal_velocity, const double desired_velocity);
  double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

private:
  double K_longtitutal_velocity_;
  double desired_velocity_;
};

LongtitutalVelocityCost::LongtitutalVelocityCost(
  const double & K_longtitutal_velocity,
  const double desired_velocity)
: Cost(), K_longtitutal_velocity_(K_longtitutal_velocity), desired_velocity_(desired_velocity)
{
}

double LongtitutalVelocityCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{
  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[1] - desired_velocity_) * K_longtitutal_velocity_;
  }

  return trajectory_cost;
}

}
}
