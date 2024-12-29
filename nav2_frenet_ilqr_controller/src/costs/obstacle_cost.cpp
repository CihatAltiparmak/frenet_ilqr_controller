#include "nav2_frenet_ilqr_controller/costs/obstacle_cost.hpp"

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

ObstacleCost::ObstacleCost()
: RclcppNodeCost()
{
}

void ObstacleCost::initialize(
  const std::string & cost_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RclcppNodeCost::initialize(cost_plugin_name, parent, costmap_ros);

  auto node = parent.lock();
  declare_parameter_if_not_declared(
    node, cost_plugin_name_ + ".K_obstacle", rclcpp::ParameterValue(2));

  node->get_parameter(
    cost_plugin_name_ + ".K_obstacle",
    K_obstacle_);
}

double ObstacleCost::cost(
  const FrenetTrajectory & /*frenet_trajectory*/,
  const CartesianTrajectory & cartesian_trajectory)
{
  double trajectory_cost = 0;

  for (auto cartesian_state : cartesian_trajectory) {
    const double & x = cartesian_state[0];
    const double & y = cartesian_state[3];
    unsigned int x_i, y_i;

    double point_cost = 0.0;
    if (costmap_->worldToMap(x, y, x_i, y_i)) {
      point_cost = costmap_->getCost(x_i, y_i) / nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    trajectory_cost += point_cost;
  }

  return K_obstacle_ * trajectory_cost / cartesian_trajectory.size();
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::ObstacleCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)
