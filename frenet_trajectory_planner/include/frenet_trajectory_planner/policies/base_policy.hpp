// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Dr. Tufan Kumbasar, ITU AI2S Lab
// Copyright (C) 2024 Prof. Dr. Behçet Uğur Töreyin
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

#include <vector>
#include <iostream>

#include <frenet_trajectory_planner/type_definitions.hpp>

namespace frenet_trajectory_planner
{
namespace policies
{

class Policy
{
public:
  Policy() {}
  virtual ~Policy() = default;
  virtual bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;
};

template<typename Parameters>
class BasePolicy : public Policy
{
public:
  explicit BasePolicy(const Parameters & parameters)
  : parameters_(parameters)
  {
  }

  Parameters getParameters() const
  {
    return parameters_;
  }

protected:
  Parameters parameters_;
};

}  // namespace policies
}  // namespace frenet_trajectory_planner
