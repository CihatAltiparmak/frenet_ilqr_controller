#pragma once

#include <frenet_trajectory_planner/costs/distance_cost.hpp>
#include <cmath>

namespace frenet_trajectory_planner
{
namespace costs
{

class LateralDistanceCost : public Cost
{
public:
  LateralDistanceCost(const double & K_distance);
  double cost(const FrenetTrajectory & frenet_trajectory) override;

private:
  double K_distance_;
};

LateralDistanceCost::LateralDistanceCost(const double & K_distance)
: Cost(), K_distance_(K_distance)
{
}

double LateralDistanceCost::cost(const FrenetTrajectory & frenet_trajectory)
{
  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[3] * K_distance_);
  }

  return trajectory_cost;
}

}
}
