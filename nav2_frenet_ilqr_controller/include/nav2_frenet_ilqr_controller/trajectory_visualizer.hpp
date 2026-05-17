// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Dr. Tufan Kumbasar, ITU AI2S Lab
// Copyright (C) 2024 Prof. Dr. Behçet Uğur Töreyin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef NAV2_FRENET_ILQR_CONTROLLER__TRAJECTORY_VISUALIZER_HPP_
#define NAV2_FRENET_ILQR_CONTROLLER__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "frenet_trajectory_planner/type_definitions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_frenet_ilqr_controller
{

class TrajectoryVisualizer {
public:
  TrajectoryVisualizer() {}

  void on_configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & frame_id);
  void on_activate();
  void on_deactivate();
  void on_cleanup();
  void reset();

  void visualize(
    const std::shared_ptr<frenet_trajectory_planner::DebugInfo> debug_info);

  inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

private:
  std::shared_ptr<nav2::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
  std::string frame_id_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_frenet_ilqr_controller

#endif  // NAV2_FRENET_ILQR_CONTROLLER__TRAJECTORY_VISUALIZER_HPP_
