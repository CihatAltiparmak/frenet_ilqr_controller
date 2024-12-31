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
    trajectory_cost += std::abs(frenet_state[1] - desired_velocity_);
  }

  return K_longtitutal_velocity_ * trajectory_cost;
}

}
}
