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
  const nav2::LifecycleNode::WeakPtr & parent,
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
  [[maybe_unused]] const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & cartesian_trajectory)
{
  double trajectory_cost = 0;

  double max_curvature = 0.0;
  for (auto c_state : cartesian_trajectory) {
    double up = std::abs(c_state[1] * c_state[5] - c_state[2] * c_state[4]);
    double down = std::hypot(c_state[1], c_state[4]);
    double curvature= up / (down * down * down);

    max_curvature = std::max(max_curvature, curvature);
  }

  trajectory_cost = max_curvature;

  // if (!frenet_trajectory.empty()) {
  //   const auto & frenet_state = frenet_trajectory.back();
  //   trajectory_cost += std::abs(frenet_state[4]) * max_curvature;
  // }

  return K_latlon_balance_ * trajectory_cost;
}

}  // namespace costs
}  // namespace nav2_frenet_ilqr_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LatLonBalanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
