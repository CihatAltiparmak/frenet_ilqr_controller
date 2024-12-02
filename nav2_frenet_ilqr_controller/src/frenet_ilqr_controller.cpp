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
#include <pluginlib/class_loader.hpp>

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT
using nav2_util::geometry_utils::euclidean_distance;

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

  parameter_handler_ = std::make_unique<nav2_frenet_ilqr_controller::ParameterHandler>(
    parent,
    plugin_name_,
    costmap_->getSizeInMetersX());
  params_ = parameter_handler_->getParams();

  addPoliciesFromPlugins();
  addCostsFromPlugins();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

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

void FrenetILQRController::addPoliciesFromPlugins()
{

  auto node = node_.lock();

  std::vector<std::string> default_policy_plugins = {};
  std::vector<std::string> policy_plugins;

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".policy_plugins", rclcpp::ParameterValue(default_policy_plugins));

  node->get_parameter(
    plugin_name_ + ".policy_plugins",
    policy_plugins);

  pluginlib::ClassLoader<policies::RclcppNodePolicy> policy_loader("nav2_frenet_ilqr_controller",
    "nav2_frenet_ilqr_controller::policies::RclcppNodePolicy");
  for (const auto & policy_plugin_name : policy_plugins) {
    std::string absolute_policy_plugin_name = plugin_name_ + ".";
    absolute_policy_plugin_name += policy_plugin_name;
    auto policy_plugin_type = nav2_util::get_plugin_type_param(node, absolute_policy_plugin_name);
    RCLCPP_INFO(
      logger_, "Policy Plugin is initializing. : %s %s",
      policy_plugin_name.c_str(), policy_plugin_type.c_str());

    std::shared_ptr<policies::RclcppNodePolicy> policy = policy_loader.createSharedInstance(
      policy_plugin_type);
    policy->initialize(policy_plugin_name, node_, costmap_ros_);
    frenet_trajectory_planner_.addPolicy(policy);
  }
}

void FrenetILQRController::addCostsFromPlugins()
{

  auto node = node_.lock();

  std::vector<std::string> default_cost_checker_plugins = {};
  std::vector<std::string> cost_checker_plugins;

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_checker_plugins",
    rclcpp::ParameterValue(default_cost_checker_plugins));

  node->get_parameter(
    plugin_name_ + ".cost_checker_plugins",
    cost_checker_plugins);

  pluginlib::ClassLoader<costs::RclcppNodeCost> cost_loader("nav2_frenet_ilqr_controller",
    "nav2_frenet_ilqr_controller::costs::RclcppNodeCost");
  for (const auto & cost_checker_plugin_name : cost_checker_plugins) {
    std::string absolute_cost_checker_plugin_name = plugin_name_ + ".";
    absolute_cost_checker_plugin_name += cost_checker_plugin_name;
    auto cost_checker_plugin_type = nav2_util::get_plugin_type_param(
      node,
      absolute_cost_checker_plugin_name);
    RCLCPP_INFO(
      logger_, "Cost Checker Plugin is initializing. : %s %s",
      cost_checker_plugin_name.c_str(), cost_checker_plugin_type.c_str());

    std::shared_ptr<costs::RclcppNodeCost> cost_checker = cost_loader.createSharedInstance(
      cost_checker_plugin_type);
    cost_checker->initialize(cost_checker_plugin_name, node_, costmap_ros_);
    frenet_trajectory_planner_.addCost(cost_checker);
  }
}

nav_msgs::msg::Path FrenetILQRController::truncateGlobalPlanWithLookAheadDist(
  const geometry_msgs::msg::PoseStamped & pose_stamped,
  const nav_msgs::msg::Path & path,
  const double lookahead_distance)
{
  // TODO (CihatAltiparmak) : find better algorithm to handle this
  size_t lookahead_index = 0;
  for (size_t index = 0; index < path.poses.size(); ++index) {
    if (euclidean_distance(
        path.poses[index].pose.position,
        pose_stamped.pose.position) < lookahead_distance)
    {
      lookahead_index = index;
    }
  }

  nav_msgs::msg::Path truncated_path;
  truncated_path.header = path.header;
  for (auto index = lookahead_index; index < path.poses.size(); ++index) {
    truncated_path.poses.push_back(path.poses[index]);
  }
  return truncated_path;
}

Vector2d FrenetILQRController::findOptimalInputForTrajectory(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
  const frenet_trajectory_planner::CartesianTrajectory & robot_cartesian_trajectory)
{

  using ilqr_trajectory_tracker::DiffDriveRobotModel;
  using ilqr_trajectory_tracker::DiffDriveRobotModelState;
  using ilqr_trajectory_tracker::DiffDriveRobotModelInput;

  std::vector<DiffDriveRobotModelState> X_feasible;
  for (auto cartesian_state : robot_cartesian_trajectory) {
    DiffDriveRobotModelState x;
    x << cartesian_state[0],
      cartesian_state[3],
      std::atan2(cartesian_state[4], cartesian_state[1]);
    X_feasible.push_back(x);
  }

  Matrix3d Q = Matrix3d::Identity() * 10;
  Matrix2d R = Matrix2d::Identity() * 2;
  double alpha = 1;
  double dt = 0.05;
  ilqr_trajectory_tracker::NewtonOptimizer<DiffDriveRobotModel> newton_optimizer;
  newton_optimizer.setIterationNumber(20);
  newton_optimizer.setAlpha(alpha);
  auto U_optimal = newton_optimizer.optimize(X_feasible, Q, R, dt);

  if (U_optimal.empty()) {
    throw nav2_core::NoValidControl("Iterative LQR couldn't find any solution!");
  }

  return U_optimal[0];
}

geometry_msgs::msg::TwistStamped FrenetILQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(parameter_handler_->getMutex());
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!path_handler_->transformPose(transformed_plan.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  double lookahead_distance = 0.3;
  // TODO (CihatAltiparmak) : we should ignore the waypoints whose angles between them equal to zero or extremely large
  transformed_plan = truncateGlobalPlanWithLookAheadDist(
    robot_pose, transformed_plan,
    lookahead_distance);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.size() < 2) {
    transformed_plan.poses.insert(transformed_plan.poses.begin(), robot_pose);
  }

  std::vector<frenet_trajectory_planner::CartesianPoint> waypoint_list;
  for (const auto & pose_stamped : transformed_plan.poses) {
    frenet_trajectory_planner::CartesianPoint point;
    point << pose_stamped.pose.position.x, pose_stamped.pose.position.y;
    waypoint_list.push_back(point);
  }

  frenet_trajectory_planner::CartesianState robot_cartesian_state =
    frenet_trajectory_planner::CartesianState::Zero();
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double linear_speed = speed.linear.x;
  if (linear_speed == 0) {
    linear_speed = 0.001;
  }
  robot_cartesian_state[0] = robot_pose.pose.position.x;
  robot_cartesian_state[1] = linear_speed * std::cos(robot_yaw);
  robot_cartesian_state[3] = robot_pose.pose.position.y;
  robot_cartesian_state[4] = linear_speed * std::sin(robot_yaw);

  frenet_trajectory_planner_.setFrenetTrajectoryPlannerConfig(
    params_->frenet_trajectory_planner_config);
  auto planned_cartesian_trajectory = frenet_trajectory_planner_.planByWaypoint(
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
  robot_pose_pub_->publish(robot_pose);

  auto u_opt = findOptimalInputForTrajectory(robot_pose, planned_cartesian_trajectory);

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = u_opt[0];
  cmd_vel.twist.angular.z = u_opt[1];
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
  std::lock_guard<std::mutex> lock_reinit(parameter_handler_->getMutex());
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
