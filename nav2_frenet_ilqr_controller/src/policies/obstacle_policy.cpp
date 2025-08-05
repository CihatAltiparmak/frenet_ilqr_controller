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

#include "nav2_frenet_ilqr_controller/policies/obstacle_policy.hpp"
#include <memory>

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

ObstaclePolicy::ObstaclePolicy()
: RclcppNodePolicy()
{
}

void ObstaclePolicy::initialize(
  const std::string & policy_plugin_name,
  const nav2::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodePolicy::initialize(policy_plugin_name, parent, costmap_ros);
  collision_checker_.setCostmap(costmap_);
}

bool ObstaclePolicy::checkIfFeasible(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{

  for (auto cartesian_state : cartesian_trajectory) {
    if (isCollides(cartesian_state)) {
      return false;
    }
  }

  return true;
}

bool ObstaclePolicy::isCollides(const CartesianState & cartesian_state)
{

  const double & x = cartesian_state[0];
  const double & y = cartesian_state[3];
  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    return false;
  }

  unsigned char point_cost = collision_checker_.pointCost(x_i, y_i);

  switch (point_cost) {
    case (nav2_costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return true;
    case (nav2_costmap_2d::NO_INFORMATION):
      return false;   // TODO (CihatAltiparmak) : we should control if it's tracking unknown
    default:
      return false;
  }
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::policies::ObstaclePolicy,
  nav2_frenet_ilqr_controller::policies::RclcppNodePolicy)
