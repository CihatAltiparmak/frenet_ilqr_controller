#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>

namespace frenet_trajectory_planner
{

struct Info {
  double arclength;
};

namespace costs
{

class Cost
{
public:
  Cost() {}
  virtual ~Cost() = default;
  virtual double cost(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;
  void setInfo(const Info & info) {
    info_ = info;
  }
protected:
    Info info_;
};

}
}
