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

  node->get_parameter(
    cost_plugin_name_ + ".K_longtitutal_velocity",
    K_longtitutal_velocity_);

  node->get_parameter(
    cost_plugin_name_ + ".desired_velocity",
    desired_velocity_);
}

double LongtitutalVelocityCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{
  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[1] - desired_velocity_);
  }

  return K_longtitutal_velocity_ * trajectory_cost;
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
