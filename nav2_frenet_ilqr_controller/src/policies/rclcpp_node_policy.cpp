#include "frenet_trajectory_planner/policies/rclcpp_node_policy.hpp"

using namespace frenet_trajectory_planner::policies;
using namespace frenet_trajectory_planner;

namespace nav2_frenet_ilqr_controller
{
namespace policies
{

RclcppNodePolicy::RclcppNodePolicy()
: Policy()
{
}

void RclcppNodePolicy::initialize(
  const std::string & policy_plugin_name,
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  policy_plugin_name_ = policy_plugin_name;
  node_ = parent;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
}

}
}