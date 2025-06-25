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

#ifndef NAV2_FRENET_ILQR_CONTROLLER__COSTS__LATERAL_DISTANCE_COST_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__COSTS__LATERAL_DISTANCE_COST_HPP_

#include <memory>
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_frenet_ilqr_controller/costs/rclcpp_node_cost.hpp"

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

class LateralDistanceCost : public RclcppNodeCost
{

public:
  LateralDistanceCost();
  void initialize(
    const std::string & cost_plugin_name,
    const nav2::LifecycleNode::WeakPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

protected:
  double K_lateral_distance_;
};

}
}

#endif
