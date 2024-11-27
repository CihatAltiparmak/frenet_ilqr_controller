#pragma once
#include <frenet_trajectory_planner/type_definitions.hpp>


namespace frenet_trajectory_planner
{

class BaseAdapter
{
public:
  BaseAdapter();
  virtual CartesianState convertFrenet2Cartesian(const FrenetState & frenet_state) = 0;
  virtual FrenetState convertCartesian2Frenet(const CartesianState & cartesian_state) = 0;
  double getArclength();

protected:
  double arclength_;
};

BaseAdapter::BaseAdapter()
{
}

double BaseAdapter::getArclength()
{
  return arclength_;
}

}
