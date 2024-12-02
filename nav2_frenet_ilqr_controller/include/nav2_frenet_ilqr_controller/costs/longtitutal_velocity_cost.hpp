#ifndef NAV2_FRENET_ILQR_CONTROLLER__COSTS__LONGTITUTAL_VELOCITY_COST_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__COSTS__LONGTITUTAL_VELOCITY_COST_HPP_

#include <memory>
#include "nav2_util/node_utils.hpp"
#include "nav2_frenet_ilqr_controller/costs/rclcpp_node_cost.hpp"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

class LongtitutalVelocityCost : public RclcppNodeCost
{

public:
  LongtitutalVelocityCost();
  void initialize(
    const std::string & cost_plugin_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

protected:
  double K_longtitutal_velocity_;
  double desired_velocity_;
};

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost,
  nav2_frenet_ilqr_controller::costs::RclcppNodeCost)

#endif
