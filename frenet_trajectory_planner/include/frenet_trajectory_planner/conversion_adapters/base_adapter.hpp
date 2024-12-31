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


namespace frenet_trajectory_planner
{

class BaseAdapter
{
public:
  BaseAdapter();
  virtual CartesianState convertFrenet2Cartesian(const FrenetState & frenet_state) = 0;
  virtual FrenetState convertCartesian2Frenet(const CartesianState & cartesian_state) = 0;
  double getArclength();

protected:
  double arclength_;
};

BaseAdapter::BaseAdapter()
{
}

double BaseAdapter::getArclength()
{
  return arclength_;
}

}
