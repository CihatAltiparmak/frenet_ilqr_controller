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
#ifndef NAV2_FRENET_ILQR_CONTROLLER__POLICIES__RCLCPP_NODE_POLICY_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__POLICIES__RCLCPP_NODE_POLICY_HPP_

#include <memory>
#include <string>
#include "frenet_trajectory_planner/policies/base_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using namespace frenet_trajectory_planner::policies;  // NOLINT
using namespace frenet_trajectory_planner;  // NOLINT

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

class RclcppNodePolicy : public Policy
{
public:
  RclcppNodePolicy();
  virtual void initialize(
    const std::string & policy_plugin_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    policy_plugin_name_ = policy_plugin_name;
    node_ = parent;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
  }
  virtual bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string policy_plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

RclcppNodePolicy::RclcppNodePolicy()
: Policy()
{
}

}  // namespace policies
}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__POLICIES__RCLCPP_NODE_POLICY_HPP_
