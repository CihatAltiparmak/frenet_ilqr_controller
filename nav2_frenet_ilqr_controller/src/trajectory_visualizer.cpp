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

#include "nav2_frenet_ilqr_controller/trajectory_visualizer.hpp"

namespace nav2_frenet_ilqr_controller
{

void TrajectoryVisualizer::on_configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & frame_id)
{
  frame_id_ = frame_id;

  nav2::LifecycleNode::SharedPtr node = parent.lock();
  clock_ = node->get_clock();
  marker_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("candidate_trajectories", 1);
}

void TrajectoryVisualizer::on_activate()
{
  marker_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
  marker_pub_->on_deactivate();
}

void TrajectoryVisualizer::on_cleanup()
{
  marker_pub_.reset();
}

void TrajectoryVisualizer::visualize(
  const frenet_trajectory_planner::DebugInfo & debug_info)
{
  auto marker_list = visualization_msgs::msg::MarkerArray();

  for (size_t i = 0; i < debug_info.cartesian_trajectories.size(); ++i) {
    const auto & cand_traj = debug_info.cartesian_trajectories[i];
    const auto & cand_cost = debug_info.costs[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = clock_->now();
    marker.ns = "Candidate Trajectories";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;      // line width
    marker.color = (cand_cost == -1.0) ?
      createColor(1.0f, 0.0f, 1.0f, 0.6f) :      // magenta for collisions
      createColor(1.0f, 1.0f, 0.0f, 0.7f);

    marker.points.reserve(cand_traj.size());
    for (const auto & cstate : cand_traj) {
      geometry_msgs::msg::Point pt;
      pt.x = cstate[0];
      pt.y = cstate[3];
      pt.z = 0.03;
      marker.points.push_back(pt);
    }

    marker_list.markers.push_back(marker);
  }

  marker_pub_->publish(marker_list);
}

}  // namespace nav2_frenet_ilqr_controller
