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
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  virtual bool check_if_feasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

RclcppNodePolicy::RclcppNodePolicy()
: Policy()
{
}

void RclcppNodePolicy::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
}

}
}

#endif
