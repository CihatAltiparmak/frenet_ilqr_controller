#ifndef NAV2_FRENET_ILQR_CONTROLLER__POLICIES__OBSTACLE_POLICY_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__POLICIES__OBSTACLE_POLICY_HPP_

#include "nav2_frenet_ilqr_controller/policies/rclcpp_node_policy.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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
    const nav2::LifecycleNode::WeakPtr & parent,
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
