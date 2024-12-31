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

#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <vector>
#include <cmath>

namespace frenet_trajectory_planner
{
namespace policies
{

typedef struct AccelerationLimits
{
  double acceleration_min;
  double acceleration_max;
} AccelerationPolicyParameters;

class AccelerationPolicy : public BasePolicy<AccelerationPolicyParameters>
{
public:
  AccelerationPolicy(
    const AccelerationPolicyParameters & acceleration_policy_parameters,
    const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter);
  bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

private:
  std::shared_ptr<FrenetFrameConverter> frenet_frame_converter_;
};

AccelerationPolicy::AccelerationPolicy(
  const AccelerationPolicyParameters & acceleration_policy_parameters,
  const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter)
: BasePolicy<AccelerationPolicyParameters>(acceleration_policy_parameters),
  frenet_frame_converter_(frenet_frame_converter)
{

}

bool AccelerationPolicy::checkIfFeasible(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & cartesian_trajectory)
{

  // assert that frenet_trajectory equals a presentation in frenet frame of cartesian_trajectory
  for (const auto & state : cartesian_trajectory) {
    double acceleration = (state({2, 5})).norm();
    if (acceleration > this->parameters_.acceleration_max ||
      acceleration < this->parameters_.acceleration_min)
    {
      return false;
    }
  }

  return true;
}

}
}
