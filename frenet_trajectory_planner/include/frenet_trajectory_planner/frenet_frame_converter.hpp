#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/conversion_adapters/base_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <vector>
#include <memory>

namespace frenet_trajectory_planner
{

class FrenetFrameConverter
{
public:
  FrenetFrameConverter();
  void createSegments(const std::vector<CartesianPoint> & waypoint_list);
  CartesianTrajectory convertFrenet2Cartesian(
    const FrenetTrajectory & frenet_trajectory);
  FrenetState convertCartesian2FrenetForSegment(
    const CartesianState & cartesian_state,
    const size_t segment_index);
  CartesianState convertFrenet2CartesianForSegment(
    const FrenetState & frenet_state,
    const size_t segment_index);

private:
  std::vector<std::unique_ptr<BaseAdapter>> segments_;
};

FrenetFrameConverter::FrenetFrameConverter()
{
}

void FrenetFrameConverter::createSegments(const std::vector<CartesianPoint> & waypoint_list)
{
  // assert that waypoint_list size must be at least 2
  CartesianPoint last_waypoint = waypoint_list[0];
  for (auto waypoint_it = waypoint_list.begin() + 1; (waypoint_it + 1) != waypoint_list.end();
    ++waypoint_it)
  {
    CartesianPoint qim1 = last_waypoint;
    CartesianPoint qi = *(waypoint_it);
    CartesianPoint qip1 = *(waypoint_it + 1);

    Vector2d yi = (qi - qim1) / (qi - qim1).norm();
    Vector2d yip1 = (qip1 - qi) / (qip1 - qi).norm();
    double alphai = std::acos(yi.dot(yip1));

    double delta = 1;
    double li = std::min(
      {(qi - qim1).norm() / 2,
        (qip1 - qi).norm() / 2,
        delta * std::sin(alphai / 2) / (1 - std::cos(alphai / 2))});
    double ri = li / std::tan(alphai / 2);
    CartesianPoint ci = qi + ((yip1 - yi) / (yip1 - yi).norm()) * (ri / std::cos(alphai / 2));
    Vector2d xi = (qi - li * yi - ci) / (qi - li * yi - ci).norm();

    // determinant calculations to determine direction
    // TODO (CihatAltiparmak) : seems it works but take a look at again later
    double chross = yip1.x() * yi.y() - yip1.y() * yi.x();
    int sign_indicactor = (chross > 0) ? 1 : -1;

    CartesianPoint new_q1 = qim1;
    CartesianPoint new_q2 = ci + ri * xi;
    CartesianPoint new_q3 = ci + ri * (xi * std::cos(alphai) + yi * std::sin(alphai));

    // if three points nearly present line, just ignore to smooth with circle adapter
    if (alphai < 0.1 || std::isnan(alphai)) {
      segments_.push_back(
        std::make_unique<LineAdapter>(new_q1, qi)
      );
      last_waypoint = qi;
    } else {
      segments_.push_back(
        std::make_unique<LineAdapter>(new_q1, new_q2)
      );

      segments_.push_back(
        std::make_unique<CircleAdapter>(new_q2, new_q3, ci, ri, xi, yi, alphai, sign_indicactor));
      last_waypoint = new_q3;
    }
  }

  segments_.push_back(
    std::make_unique<LineAdapter>(last_waypoint, waypoint_list.back()));

}

CartesianTrajectory FrenetFrameConverter::convertFrenet2Cartesian(
  const FrenetTrajectory & frenet_trajectory)
{
  double current_longitutal_length = 0;
  size_t current_segment_index = 0;
  CartesianTrajectory cartesian_trajectory;
  for (auto frenet_state : frenet_trajectory) {
    while (frenet_state[0] >
      current_longitutal_length + segments_.at(current_segment_index)->getArclength())
    {
      current_longitutal_length += segments_.at(current_segment_index)->getArclength();
      ++current_segment_index;

      if (current_segment_index >= segments_.size()) {
        return cartesian_trajectory;
      }
    }

    FrenetState converted_frenet_state = frenet_state;
    converted_frenet_state[0] -= current_longitutal_length;
    CartesianState cartesian_state =
      segments_.at(current_segment_index)->convertFrenet2Cartesian(converted_frenet_state);
    cartesian_trajectory.push_back(cartesian_state);
  }

  return cartesian_trajectory;
}

FrenetState FrenetFrameConverter::convertCartesian2FrenetForSegment(
  const CartesianState & cartesian_state, const size_t segment_index)
{
  return segments_.at(segment_index)->convertCartesian2Frenet(cartesian_state);
}

CartesianState FrenetFrameConverter::convertFrenet2CartesianForSegment(
  const FrenetState & frenet_state, const size_t segment_index)
{
  return segments_.at(segment_index)->convertFrenet2Cartesian(frenet_state);
}
}
