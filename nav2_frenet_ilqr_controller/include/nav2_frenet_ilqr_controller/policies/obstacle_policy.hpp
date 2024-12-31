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

#ifndef NAV2_FRENET_ILQR_CONTROLLER__POLICIES__OBSTACLE_POLICY_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__POLICIES__OBSTACLE_POLICY_HPP_

#include "nav2_frenet_ilqr_controller/policies/rclcpp_node_policy.hpp"
#include <memory>

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

class ObstaclePolicy : public RclcppNodePolicy
{

public:
  ObstaclePolicy();
  void initialize(
    const std::string & policy_plugin_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

  bool isCollides(const CartesianState & cartesian_state);

private:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_{
    nullptr};
};

}
}

#endif
