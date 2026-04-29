<<<<<<< HEAD
=======
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

>>>>>>> 4094837 (Format code according to ros standard (#71))
#include "nav2_frenet_ilqr_controller/costs/lateral_distance_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

LateralDistanceCost::LateralDistanceCost()
: RclcppNodeCost()
{
}

void LateralDistanceCost::initialize(
  const std::string & cost_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();
  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_lateral_distance", rclcpp::ParameterValue(10.0));

  node->get_parameter(
    cost_plugin_name_ + ".K_lateral_distance",
    K_lateral_distance_);
}

double LateralDistanceCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{
  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[3]);
  }

  return K_lateral_distance_ * trajectory_cost;
}

}  // namespace costs
}  // namespace nav2_frenet_ilqr_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LateralDistanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
