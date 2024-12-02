#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>

namespace frenet_trajectory_planner
{
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
};

}
}
