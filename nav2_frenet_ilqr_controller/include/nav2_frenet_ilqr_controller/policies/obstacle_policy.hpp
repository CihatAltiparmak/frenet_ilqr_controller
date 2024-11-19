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
  bool check_if_feasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

  bool is_collides(const CartesianState & cartesian_state);
};

ObstaclePolicy::ObstaclePolicy()
: RclcppNodePolicy()
{
}

bool ObstaclePolicy::check_if_feasible(
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
  if (!costmap_->worldToMap(x, y, x_i, y_i)) {
    return false;
  }

  unsigned char point_cost = costmap_->getCost(x_i, y_i);

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
