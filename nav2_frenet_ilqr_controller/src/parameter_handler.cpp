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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <iostream>

#include "nav2_frenet_ilqr_controller/parameter_handler.hpp"

namespace nav2_frenet_ilqr_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & plugin_name, const double costmap_size_x_in_meters)
: plugin_name_(plugin_name)
{
  node_ = parent;
  auto node = node_.lock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".interpolate_curvature_after_goal", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(costmap_size_x_in_meters / 2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".time_discretization", rclcpp::ParameterValue(0.05));

  // for lateral distance
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.min_lateral_distance",
    rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.max_lateral_distance",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.step_lateral_distance",
    rclcpp::ParameterValue(0.5));

  // for longtitutal velocity
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.min_longtitutal_velocity", rclcpp::ParameterValue(
      0.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.max_longtitutal_velocity", rclcpp::ParameterValue(
      2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.step_longtitutal_velocity", rclcpp::ParameterValue(
      0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.time_interval", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".frenet_trajectory_planner.max_state_in_trajectory", rclcpp::ParameterValue(
      2));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".ilqr_trajectory_tracker.iteration_number", rclcpp::ParameterValue(20));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".ilqr_trajectory_tracker.alpha", rclcpp::ParameterValue(1.0));


  declare_parameter_if_not_declared(
    node, plugin_name_ + ".ilqr_trajectory_tracker.input_limits_min", rclcpp::ParameterValue(std::vector<double>({0.0, -1.5})));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".ilqr_trajectory_tracker.input_limits_max", rclcpp::ParameterValue(std::vector<double>({1.0, 1.5})));
  node->get_parameter(
    plugin_name_ + ".interpolate_curvature_after_goal",
    params_.interpolate_curvature_after_goal);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    params_.max_robot_pose_search_dist);
  node->get_parameter(plugin_name_ + ".transform_tolerance", params_.transform_tolerance);

  node->get_parameter(plugin_name_ + ".time_discretization", params_.time_discretization);
  params_.frenet_trajectory_planner_config.dt = params_.time_discretization;

  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.min_lateral_distance",
    params_.frenet_trajectory_planner_config.min_lateral_distance);
  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.max_lateral_distance",
    params_.frenet_trajectory_planner_config.max_lateral_distance);
  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.step_lateral_distance",
    params_.frenet_trajectory_planner_config.step_lateral_distance);

  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.min_longtitutal_velocity",
    params_.frenet_trajectory_planner_config.min_longtitutal_velocity);
  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.max_longtitutal_velocity",
    params_.frenet_trajectory_planner_config.max_longtitutal_velocity);
  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.step_longtitutal_velocity",
    params_.frenet_trajectory_planner_config.step_longtitutal_velocity);

  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.time_interval",
    params_.frenet_trajectory_planner_config.time_interval);
  node->get_parameter(
    plugin_name_ + ".frenet_trajectory_planner.max_state_in_trajectory",
    params_.frenet_trajectory_planner_config.max_state_in_trajectory);

  node->get_parameter(
    plugin_name_ + ".ilqr_trajectory_tracker.iteration_number",
    params_.iteration_number);
  node->get_parameter(plugin_name_ + ".ilqr_trajectory_tracker.alpha", params_.alpha);


  {
    std::vector<double> input_limits_min;
    node->get_parameter(
      plugin_name_ + ".ilqr_trajectory_tracker.input_limits_min", input_limits_min);
    params_.input_limits_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_limits_min.data(), input_limits_min.size());
  }
  {
    std::vector<double> input_limits_max;
    node->get_parameter(
      plugin_name_ + ".ilqr_trajectory_tracker.input_limits_max", input_limits_max);
    params_.input_limits_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_limits_max.data(), input_limits_max.size());
  }
  dynamic_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::dynamicParametersCallback,
      this, std::placeholders::_1));
}

ParameterHandler::~ParameterHandler()
{
  auto node = node_.lock();
  if (dynamic_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dynamic_params_handler_.get());
  }
  dynamic_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == plugin_name_ + ".interpolate_curvature_after_goal") {
      params_.interpolate_curvature_after_goal = parameter.as_double();
    } else if (name == plugin_name_ + ".max_robot_pose_search_dist") {
      params_.max_robot_pose_search_dist = parameter.as_double();
    } else if (name == plugin_name_ + ".transform_tolerance") {
      params_.transform_tolerance = parameter.as_double();
    } else if (name == plugin_name_ + ".time_discretization") {
      params_.time_discretization = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.min_lateral_distance") {
      params_.frenet_trajectory_planner_config.min_lateral_distance = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.max_lateral_distance") {
      params_.frenet_trajectory_planner_config.max_lateral_distance = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.step_lateral_distance") {
      params_.frenet_trajectory_planner_config.step_lateral_distance = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.min_longtitutal_velocity") {
      params_.frenet_trajectory_planner_config.min_longtitutal_velocity = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.max_longtitutal_velocity") {
      params_.frenet_trajectory_planner_config.max_longtitutal_velocity = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.step_longtitutal_velocity") {
      params_.frenet_trajectory_planner_config.step_longtitutal_velocity = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.time_interval") {
      params_.frenet_trajectory_planner_config.time_interval = parameter.as_double();
    } else if (name == plugin_name_ + ".frenet_trajectory_planner.max_state_in_trajectory") {
      if (parameter.as_int() < 1) {
        params_.frenet_trajectory_planner_config.max_state_in_trajectory = 2;
      } else {
        params_.frenet_trajectory_planner_config.max_state_in_trajectory = parameter.as_int();
      }
    } else if (name == plugin_name_ + ".ilqr_trajectory_tracker.iteration_number") {
      if (parameter.as_int() < 0) {
        params_.iteration_number = 20;
      } else {
        params_.iteration_number = parameter.as_int();
      }
    } else if (name == plugin_name_ + ".ilqr_trajectory_tracker.alpha") {
      params_.alpha = parameter.as_double();
    }
    else if (name == plugin_name_ + ".ilqr_trajectory_tracker.input_limits_min") {
    auto input_limits_min = parameter.as_double_array();
    params_.input_limits_min = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_limits_min.data(), input_limits_min.size());
  } else if (name == plugin_name_ + ".ilqr_trajectory_tracker.input_limits_max") {
    auto input_limits_max = parameter.as_double_array();
    params_.input_limits_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_limits_max.data(), input_limits_max.size());
  }
}

  result.successful = true;
  return result;
}

}  // namespace nav2_frenet_ilqr_controller
