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
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

  bool is_collides(const CartesianState & cartesian_state);

private:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_{
    nullptr};
};

ObstaclePolicy::ObstaclePolicy()
: RclcppNodePolicy()
{
}

void ObstaclePolicy::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodePolicy::initialize(node, costmap_ros);
  collision_checker_.setCostmap(costmap_);
}

bool ObstaclePolicy::checkIfFeasible(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{

  for (auto cartesian_state : cartesian_trajectory) {
    if (is_collides(cartesian_state)) {
      return false;
    }
  }

  return true;
}

bool ObstaclePolicy::is_collides(const CartesianState & cartesian_state)
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

#endif
