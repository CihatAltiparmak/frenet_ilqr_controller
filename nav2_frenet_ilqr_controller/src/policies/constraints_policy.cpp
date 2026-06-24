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

#include <memory>
#include "nav2_frenet_ilqr_controller/policies/constraints_policy.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

ConstraintsPolicy::ConstraintsPolicy()
: RclcppNodePolicy()
{
}

void ConstraintsPolicy::initialize(
  const std::string & policy_plugin_name,
  const nav2::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodePolicy::initialize(policy_plugin_name, parent, costmap_ros);

  auto node = parent.lock();

  declare_parameter_if_not_declared(
    node, policy_plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, policy_plugin_name_ + ".max_acceleration", rclcpp::ParameterValue(2.5));

  node->get_parameter(
    policy_plugin_name_ + ".max_speed",
    max_speed_);

  node->get_parameter(
    policy_plugin_name_ + ".max_acceleration",
    max_acceleration_);
}

bool ConstraintsPolicy::checkIfFeasible(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{
  for (auto c_state : cartesian_trajectory) {
    double speed = std::hypot(c_state[1], c_state[4]);
    if (speed > max_speed_) {
      return false;
    }

    double accel = std::hypot(c_state[2], c_state[5]);
    if (accel > max_acceleration_) {
      return false;
    }
  }

  return true;
}

}  // namespace policies
}  // namespace nav2_frenet_ilqr_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::policies::ConstraintsPolicy,
  nav2_frenet_ilqr_controller::policies::RclcppNodePolicy)
