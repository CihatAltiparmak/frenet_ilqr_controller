#ifndef NAV2_FRENET_ILQR_CONTROLLER__POLICIES__RCLCPP_NODE_POLICY_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__POLICIES__RCLCPP_NODE_POLICY_HPP_

#include "frenet_trajectory_planner/policies/base_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using namespace frenet_trajectory_planner::policies;
using namespace frenet_trajectory_planner;

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

class RclcppNodePolicy : public Policy
{
public:
  RclcppNodePolicy();
  virtual void initialize(
    const std::string & policy_plugin_name,
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    policy_plugin_name_ = policy_plugin_name;
    node_ = parent;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
  }
  virtual bool checkIfFeasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string policy_plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

RclcppNodePolicy::RclcppNodePolicy()
: Policy()
{
}

}
}

#endif
