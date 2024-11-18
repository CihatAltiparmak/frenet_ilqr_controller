#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "angles/angles.h"
#include "nav2_frenet_ilqr_controller/frenet_ilqr_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_frenet_ilqr_controller
{

void FrenetILQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles global path transformations
  double param_transform_tolerance = 0.1;
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(param_transform_tolerance), tf_, costmap_ros_);

  double control_frequency = 20.0;
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_plan", 1);
  truncated_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("truncated_plan", 1);
  robot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_test_pose", 1);
}

void FrenetILQRController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " frenet_ilqr_controller::FrenetILQRController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  truncated_path_pub_.reset();
  robot_pose_pub_.reset();
}

void FrenetILQRController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "frenet_ilqr_controller::FrenetILQRController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  truncated_path_pub_->on_activate();
  robot_pose_pub_->on_activate();
}

void FrenetILQRController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "frenet_ilqr_controller::FrenetILQRController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  truncated_path_pub_->on_deactivate();
  robot_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped FrenetILQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // // Transform path to robot base frame
  double param_max_robot_pose_search_dist = costmap_->getSizeInMetersX() / 2;
  bool param_interpolate_curvature_after_goal = false;
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, param_max_robot_pose_search_dist, param_interpolate_curvature_after_goal);
  global_path_pub_->publish(transformed_plan);

  std::vector<frenet_trajectory_planner::CartesianPoint> waypoint_list;
  for (const auto & pose_stamped : transformed_plan.poses) {
    frenet_trajectory_planner::CartesianPoint point;
    point << pose_stamped.pose.position.x, pose_stamped.pose.position.y;
    waypoint_list.push_back(point);
  }

  frenet_trajectory_planner::CartesianState robot_cartesian_state =
    frenet_trajectory_planner::CartesianState::Zero();
  // double robot_yaw = tf2::getYaw(pose.pose.orientation);
  RCLCPP_INFO(
    logger_, "JARBAY NIGHTCORE DEBUG : %f",
    nav2_util::geometry_utils::euclidean_distance(
      pose.pose.position,
      transformed_plan.poses[0].pose.position));
  robot_cartesian_state[0] = pose.pose.position.x;
  robot_cartesian_state[1] = 1;
  robot_cartesian_state[3] = pose.pose.position.y;
  robot_cartesian_state[4] = 1; // 0.001 * std::tan(robot_yaw);

  auto frenet_trajectory_planner = frenet_trajectory_planner::FrenetTrajectoryPlanner();
  auto planned_cartesian_trajectory = frenet_trajectory_planner.plan_by_waypoint(
    robot_cartesian_state,
    waypoint_list, 1.0);

  nav_msgs::msg::Path frenet_plan;
  frenet_plan.header = transformed_plan.header;
  for (const auto & cartesian_state : planned_cartesian_trajectory) {
    geometry_msgs::msg::PoseStamped fre_pose;
    fre_pose.pose.position.x = cartesian_state[0];
    fre_pose.pose.position.y = cartesian_state[3];

    frenet_plan.poses.push_back(fre_pose);
  }
  truncated_path_pub_->publish(frenet_plan);
  robot_pose_pub_->publish(pose);

  // throw nav2_core::NoValidControl("FrenetILQRController detected collision ahead!");

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  return cmd_vel;
}

bool FrenetILQRController::cancel()
{
  return true;
}

void FrenetILQRController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void FrenetILQRController::setSpeedLimit(
  const double & /*speed_limit*/,
  const bool & /*percentage*/)
{
  return;
}

void FrenetILQRController::reset()
{
}

}  // namespace nav2_frenet_ilqr_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_frenet_ilqr_controller::FrenetILQRController,
  nav2_core::Controller)
