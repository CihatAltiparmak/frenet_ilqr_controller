#ifndef NAV2_FRENET_ILQR_CONTROLLER__COSTS__LATERAL_DISTANCE_COST_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__COSTS__LATERAL_DISTANCE_COST_HPP_

#include <memory>
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_frenet_ilqr_controller/costs/rclcpp_node_cost.hpp"

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

class LateralDistanceCost : public RclcppNodeCost
{

public:
  LateralDistanceCost();
  void initialize(
    const std::string & cost_plugin_name,
    const nav2::LifecycleNode::WeakPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

protected:
  double K_lateral_distance_;
};

}
}

#endif
