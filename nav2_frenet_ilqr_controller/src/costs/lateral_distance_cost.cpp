#include "nav2_frenet_ilqr_controller/costs/lateral_distance_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

LateralDistanceCost::LateralDistanceCost()
: RclcppNodeCost()
{
}

void LateralDistanceCost::initialize(
  const std::string & cost_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();
  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_lateral_distance", rclcpp::ParameterValue(10.0));

  node->get_parameter(
    cost_plugin_name_ + ".K_lateral_distance",
    K_lateral_distance_);
}

double LateralDistanceCost::cost(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & /*cartesian_trajectory*/)
{

  double trajectory_cost = 0;

  for (auto frenet_state : frenet_trajectory) {
    trajectory_cost += std::abs(frenet_state[3] * K_lateral_distance_);
  }

  return trajectory_cost;
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LateralDistanceCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
