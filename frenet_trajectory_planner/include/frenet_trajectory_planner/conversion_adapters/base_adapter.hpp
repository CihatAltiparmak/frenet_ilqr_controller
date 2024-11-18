#pragma once
#include <frenet_trajectory_planner/type_definitions.hpp>


namespace frenet_trajectory_planner
{

class BaseAdapter
{
public:
  BaseAdapter();
  virtual CartesianState convert_frenet2cartesian(const FrenetState & frenet_state) = 0;
  virtual FrenetState convert_cartesian2frenet(const CartesianState & cartesian_state) = 0;
  virtual Vector2d get_x0() {return Vector2d::Zero();}
  virtual Vector2d get_t_frenet() {return Vector2d::Zero();}
  double get_arclength();
  CartesianPoint get_start_point() {return start_point_;}
  CartesianPoint get_end_point() {return end_point_;}

protected:
  double arclength_;
  CartesianPoint start_point_;
  CartesianPoint end_point_;
};

BaseAdapter::BaseAdapter()
{
}

double BaseAdapter::get_arclength()
{
  return arclength_;
}

}
