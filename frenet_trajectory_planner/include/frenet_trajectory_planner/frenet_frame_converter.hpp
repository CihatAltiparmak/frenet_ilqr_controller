#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/conversion_adapters/base_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <vector>
#include <memory>
#include <iostream>

namespace frenet_trajectory_planner
{

class FrenetFrameConverter
{
public:
  FrenetFrameConverter();
  void create_segments(const std::vector<CartesianPoint> & waypoint_list);
  CartesianTrajectory convert_frenet2cartesian(
    const FrenetTrajectory & frenet_trajectory,
    const size_t starting_segment_index = 0);
  FrenetState convert_cartesian2frenet_for_segment(
    const CartesianState & cartesian_state,
    const size_t segment_index);
  CartesianState convert_frenet2cartesian_for_segment(
    const FrenetState & frenet_state,
    const size_t segment_index);
  FrenetState convert_cartesian2frenet(
    const CartesianState & cartesian_state);

private:
  std::vector<std::unique_ptr<BaseAdapter>> segments_;
};

FrenetFrameConverter::FrenetFrameConverter()
{
}

void FrenetFrameConverter::create_segments(const std::vector<CartesianPoint> & waypoint_list)
{
  // assert that waypoint_list size must be at least 2
  CartesianPoint last_waypoint = waypoint_list[0];
  for (auto waypoint_it = waypoint_list.begin() + 1; (waypoint_it + 1) != waypoint_list.end();
    waypoint_it++)
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
    CartesianPoint new_q4 = qip1;

    segments_.push_back(
      std::make_unique<LineAdapter>(new_q1, new_q2)
    );

    segments_.push_back(
      std::make_unique<CircleAdapter>(new_q2, new_q3, ci, ri, xi, yi, alphai, sign_indicactor));

    last_waypoint = new_q3;
  }

  segments_.push_back(
    std::make_unique<LineAdapter>(last_waypoint, waypoint_list.back()));

}

CartesianTrajectory FrenetFrameConverter::convert_frenet2cartesian(
  const FrenetTrajectory & frenet_trajectory, const size_t starting_segment_index)
{
  double current_longitutal_length = 0;
  size_t current_segment_index = starting_segment_index;
  CartesianTrajectory cartesian_trajectory;
  for (auto frenet_state : frenet_trajectory) {
    if (frenet_state[0] <
      current_longitutal_length + segments_[current_segment_index]->get_arclength())
    {
      FrenetState converted_frenet_state = frenet_state;
      converted_frenet_state[0] -= current_longitutal_length;
      CartesianState cartesian_state =
        segments_[current_segment_index]->convert_frenet2cartesian(converted_frenet_state);
      cartesian_trajectory.push_back(cartesian_state);
    } else {
      current_longitutal_length += segments_[current_segment_index]->get_arclength();
      current_segment_index++;
    }

    if (current_segment_index >= segments_.size()) {
      break;
    }
  }
  return cartesian_trajectory;
}

FrenetState FrenetFrameConverter::convert_cartesian2frenet_for_segment(
  const CartesianState & cartesian_state, const size_t segment_index)
{
  // std::cout << segments_.at(segment_index)->get_x0() << std::endl
  //           << "###" << std::endl
  //           << segments_.at(segment_index)->get_t_frenet() << std::endl
  //           << "***************" << std::endl;
  return segments_.at(segment_index)->convert_cartesian2frenet(cartesian_state);
}

CartesianState FrenetFrameConverter::convert_frenet2cartesian_for_segment(
  const FrenetState & frenet_state, const size_t segment_index)
{
  return segments_.at(segment_index)->convert_frenet2cartesian(frenet_state);
}

FrenetState FrenetFrameConverter::convert_cartesian2frenet(
  const CartesianState & cartesian_state)
{

  FrenetState closest_frenet_state = FrenetState::Zero();
  double closest_dist = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < segments_.size(); index++) {
    FrenetState frenet_state = segments_.at(index)->convert_cartesian2frenet(cartesian_state);

    CartesianPoint start_point = segments_.at(index)->get_start_point();
    double dx = start_point[0] - cartesian_state[0];
    double dy = start_point[1] - cartesian_state[3];
    double lookahead_distance = 1;
    if (std::sqrt(dx * dx + dy * dy) < lookahead_distance) {
      closest_frenet_state = frenet_state;
    }

    // if (std::sqrt(frenet_state[0] * frenet_state[0]) < closest_dist) {
    //   closest_frenet_state = frenet_state;
    //   closest_dist = std::sqrt(frenet_state[0] * frenet_state[0]);
    // }
  }

  return closest_frenet_state;
}

// --------------------------------

template<typename ConversionAdapter>
class Frenet2CartesianConverter
{
public:
  template<typename ... ConversionAdapterArgs>
  Frenet2CartesianConverter(const ConversionAdapterArgs & ... conversion_adapter_args);

  CartesianState convert_state(const FrenetState & frenet_state);
  CartesianTrajectory convert_trajectory(const FrenetTrajectory & frenet_trajectory);
  std::vector<CartesianTrajectory> convert_trajectory_list(
    const std::vector<FrenetTrajectory> & frenet_trajectory_list);

private:
  ConversionAdapter conversion_adapter_;
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
Frenet2CartesianConverter<ConversionAdapter>::Frenet2CartesianConverter(
  const ConversionAdapterArgs & ... conversion_adapter_args)
: conversion_adapter_(conversion_adapter_args ...)
{

}

template<typename ConversionAdapter>
CartesianState Frenet2CartesianConverter<ConversionAdapter>::convert_state(
  const FrenetState & frenet_state)
{
  CartesianState cartesian_state = conversion_adapter_.convert_frenet2cartesian(frenet_state);

  return cartesian_state;
}

template<typename ConversionAdapter>
CartesianTrajectory Frenet2CartesianConverter<ConversionAdapter>::convert_trajectory(
  const FrenetTrajectory & frenet_trajectory)
{
  CartesianTrajectory cartesian_trajectory;
  for (const auto & frenet_state : frenet_trajectory) {
    auto cartesian_state = convert_state(frenet_state);
    cartesian_trajectory.push_back(cartesian_state);
  }

  return cartesian_trajectory;
}

template<typename ConversionAdapter>
std::vector<CartesianTrajectory> Frenet2CartesianConverter<ConversionAdapter>::
convert_trajectory_list(const std::vector<FrenetTrajectory> & frenet_trajectory_list)
{
  std::vector<CartesianTrajectory> cartesian_trajectory_list;

  for (const auto & frenet_trajectory : frenet_trajectory_list) {
    CartesianTrajectory cartesian_trajectory = convert_trajectory(frenet_trajectory);
    cartesian_trajectory_list.push_back(cartesian_trajectory);
  }

  return cartesian_trajectory_list;
}

// ----------------------------------

template<typename ConversionAdapter>
class Cartesian2FrenetConverter
{
public:
  template<typename ... ConversionAdapterArgs>
  Cartesian2FrenetConverter(const ConversionAdapterArgs & ... conversion_adapter_args);

  FrenetState convert_state(const CartesianState & cartesian_state);
  FrenetTrajectory convert_trajectory(const CartesianTrajectory & cartesian_trajectory);
  std::vector<FrenetTrajectory> convert_trajectory_list(
    const std::vector<CartesianTrajectory> & cartesian_trajectory_list);

private:
  ConversionAdapter conversion_adapter_;
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
Cartesian2FrenetConverter<ConversionAdapter>::Cartesian2FrenetConverter(
  const ConversionAdapterArgs & ... conversion_adapter_args)
: conversion_adapter_(conversion_adapter_args ...)
{

}

template<typename ConversionAdapter>
FrenetState Cartesian2FrenetConverter<ConversionAdapter>::convert_state(
  const CartesianState & cartesian_state)
{
  FrenetState frenet_state = conversion_adapter_.convert_cartesian2frenet(cartesian_state);

  return frenet_state;
}

template<typename ConversionAdapter>
FrenetTrajectory Cartesian2FrenetConverter<ConversionAdapter>::convert_trajectory(
  const CartesianTrajectory & cartesian_trajectory)
{
  FrenetTrajectory frenet_trajectory;
  for (const auto & cartesian_state : cartesian_trajectory) {
    auto frenet_state = convert_state(cartesian_state);
    frenet_trajectory.push_back(frenet_state);
  }

  return frenet_trajectory;
}

template<typename ConversionAdapter>
std::vector<FrenetTrajectory> Cartesian2FrenetConverter<ConversionAdapter>::convert_trajectory_list(
  const std::vector<CartesianTrajectory> & cartesian_trajectory_list)
{
  std::vector<FrenetTrajectory> frenet_trajectory_list;

  for (const auto & cartesian_trajectory : cartesian_trajectory_list) {
    FrenetTrajectory frenet_trajectory = convert_trajectory(cartesian_trajectory);
    frenet_trajectory_list.push_back(frenet_trajectory);
  }

  return frenet_trajectory_list;
}

}
