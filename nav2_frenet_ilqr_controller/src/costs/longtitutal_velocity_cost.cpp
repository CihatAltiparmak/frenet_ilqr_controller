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

#include "nav2_frenet_ilqr_controller/costs/longtitutal_velocity_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

LongtitutalVelocityCost::LongtitutalVelocityCost()
: RclcppNodeCost()
{
}

void LongtitutalVelocityCost::initialize(
  const std::string & cost_plugin_name,
  const nav2::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();
  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_longtitutal_velocity", rclcpp::ParameterValue(5.0));

  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".desired_velocity", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".distance_to_approach", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".desired_velocity_to_approach", rclcpp::ParameterValue(0.1));

  node->get_parameter(
    cost_plugin_name_ + ".K_longtitutal_velocity",
    K_longtitutal_velocity_);

  node->get_parameter(
    cost_plugin_name_ + ".desired_velocity",
    desired_velocity_);

  node->get_parameter(
    cost_plugin_name_ + ".distance_to_approach",
    distance_to_approach_);

  node->get_parameter(
    cost_plugin_name_ + ".desired_velocity_to_approach",
    desired_velocity_to_approach_);
}

double LongtitutalVelocityCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{
  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    if (info_.arclength - frenet_state[0] < distance_to_approach_) {
      trajectory_cost += std::abs(frenet_state[1] - desired_velocity_to_approach_);
    } else {
      trajectory_cost += std::abs(frenet_state[1] - desired_velocity_);
    }
  }

  return K_longtitutal_velocity_ * trajectory_cost / frenet_trajectory.size();
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
