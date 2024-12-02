#ifndef NAV2_FRENET_ILQR_CONTROLLER__FRENET_ILQR_CONTROLLER_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__FRENET_ILQR_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_frenet_ilqr_controller/path_handler.hpp"
#include "nav2_frenet_ilqr_controller/parameter_handler.hpp"
#include "nav2_frenet_ilqr_controller/policies/rclcpp_node_policy.hpp"
#include "nav2_frenet_ilqr_controller/costs/rclcpp_node_cost.hpp"
#include "frenet_trajectory_planner/type_definitions.hpp"
#include "frenet_trajectory_planner/frenet_trajectory_planner.hpp"
#include "ilqr_trajectory_tracker/models/diff_robot_model.hpp"
#include "ilqr_trajectory_tracker/ilqr_optimizer.hpp"

namespace nav2_frenet_ilqr_controller
{

/**
 * @class nav2_frenet_ilqr_controller::FrenetILQRController
 * @brief Regulated pure pursuit controller plugin
 */
class FrenetILQRController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_frenet_ilqr_controller::FrenetILQRController
   */
  FrenetILQRController() = default;

  /**
   * @brief Destrructor for nav2_frenet_ilqr_controller::FrenetILQRController
   */
  ~FrenetILQRController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  nav_msgs::msg::Path truncateGlobalPlanWithLookAheadDist(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const nav_msgs::msg::Path & path,
    const double lookahead_distance);

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  Vector2d findOptimalInputForTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const frenet_trajectory_planner::CartesianTrajectory & robot_cartesian_trajectory);

  bool cancel() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void reset() override;

  void addPoliciesFromPlugins();
  void addCostsFromPlugins();

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("FrenetILQRController")};

  double goal_dist_tol_;
  double control_duration_;
  bool cancelling_ = false;
  bool finished_cancelling_ = false;
  bool is_rotating_to_heading_ = false;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> truncated_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
  robot_pose_pub_;
  std::unique_ptr<nav2_frenet_ilqr_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_frenet_ilqr_controller::ParameterHandler> parameter_handler_;
  frenet_trajectory_planner::FrenetTrajectoryPlanner frenet_trajectory_planner_;
  Parameters * params_;
};

}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__FRENET_ILQR_CONTROLLER_HPP_
