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

#include "nav2_frenet_ilqr_controller/costs/latlon_balance_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

LatLonBalanceCost::LatLonBalanceCost()
: RclcppNodeCost()
{
}

void LatLonBalanceCost::initialize(
  const std::string & cost_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();
  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_latlon_balance", rclcpp::ParameterValue(10.0));

  node->get_parameter(
    cost_plugin_name_ + ".K_latlon_balance",
    K_latlon_balance_);
}

double LatLonBalanceCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{
  double trajectory_cost = 0;
  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[1] * frenet_state[4]);
  }

  return K_latlon_balance_ * trajectory_cost;
}

}  // namespace costs
}  // namespace nav2_frenet_ilqr_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LatLonBalanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
