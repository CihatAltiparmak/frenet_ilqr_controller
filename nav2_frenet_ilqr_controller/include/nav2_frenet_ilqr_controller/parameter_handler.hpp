// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_FRENET_ILQR_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "frenet_trajectory_planner/type_definitions.hpp"
#include <iostream>

struct Parameters
{
  bool interpolate_curvature_after_goal;
  double max_robot_pose_search_dist;
  double transform_tolerance;
  double time_discretization;
  int iteration_number;
  double alpha;
  frenet_trajectory_planner::FrenetTrajectoryPlannerConfig frenet_trajectory_planner_config;
  Eigen::VectorXd input_limits_min;
  Eigen::VectorXd input_limits_max;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

namespace nav2_frenet_ilqr_controller
{

/**
 * @class nav2_frenet_ilqr_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_frenet_ilqr_controller::ParameterHandler
   */
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node, const std::string & plugin_name,
    const double costmap_size_x_in_meters);

  /**
   * @brief Destrructor for nav2_frenet_ilqr_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  Parameters params_;
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dynamic_params_handler_;
};

}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__PARAMETER_HANDLER_HPP_
