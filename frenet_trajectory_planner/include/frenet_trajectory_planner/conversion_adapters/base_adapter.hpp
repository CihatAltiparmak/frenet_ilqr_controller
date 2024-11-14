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
};

BaseAdapter::BaseAdapter()
{
}

}
