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

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <vector>
#include <iostream>


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
  BasePolicy(const Parameters & parameters)
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

}
}
