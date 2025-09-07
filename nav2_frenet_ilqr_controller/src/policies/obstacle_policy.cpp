#include "nav2_frenet_ilqr_controller/policies/obstacle_policy.hpp"
#include <memory>

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

ObstaclePolicy::ObstaclePolicy()
: RclcppNodePolicy()
{
}

void ObstaclePolicy::initialize(
  const std::string & policy_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodePolicy::initialize(policy_plugin_name, parent, costmap_ros);
  collision_checker_.setCostmap(costmap_);
}

bool ObstaclePolicy::checkIfFeasible(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{

  for (auto cartesian_state : cartesian_trajectory) {
    if (isCollides(cartesian_state)) {
      return false;
    }
  }

  return true;
}

bool ObstaclePolicy::isCollides(const CartesianState & cartesian_state)
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
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
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
