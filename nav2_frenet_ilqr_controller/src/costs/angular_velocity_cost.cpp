#include "nav2_frenet_ilqr_controller/costs/angular_velocity_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

AngularVelocityCost::AngularVelocityCost()
: RclcppNodeCost()
{
}

void AngularVelocityCost::initialize(
  const std::string & cost_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();

  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_angular_velocity", rclcpp::ParameterValue(2.0));

  node->get_parameter(
    cost_plugin_name_ + ".K_angular_velocity",
    K_angular_velocity_);

  RCLCPP_INFO(
    node->get_logger(), "%s.K_angular_velocity : %f",
    cost_plugin_name_.c_str(), K_angular_velocity_);
}

double AngularVelocityCost::cost(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{
  double trajectory_cost = 0;

  if (cartesian_trajectory.size() < 2) {
    return 0.0;
  }

  for (auto cartesian_state_it = cartesian_trajectory.begin();
    cartesian_state_it != std::prev(cartesian_trajectory.end()); ++cartesian_state_it)
  {
    const double & theta1 = (*cartesian_state_it)[6];
    const double & theta2 = (*std::next(cartesian_state_it))[6];

    trajectory_cost += std::abs(theta1 - theta2);
  }

  return K_angular_velocity_ * trajectory_cost / cartesian_trajectory.size();
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::AngularVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
