#ifndef NAV2_FRENET_ILQR_CONTROLLER__COSTS__RCLCPP_NODE_COST_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__COSTS__RCLCPP_NODE_COST_HPP_

#include "frenet_trajectory_planner/costs/base_cost.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using namespace frenet_trajectory_planner::costs;
using namespace frenet_trajectory_planner;

namespace nav2_frenet_ilqr_controller
{
namespace costs
{

class RclcppNodeCost : public Cost
{
public:
  RclcppNodeCost()
  : Cost() {}
  virtual void initialize(
    const std::string & cost_plugin_name,
    const nav2::LifecycleNode::WeakPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    cost_plugin_name_ = cost_plugin_name;
    node_ = node;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
  }

protected:
  std::string cost_plugin_name_;
  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

}
}

#endif
