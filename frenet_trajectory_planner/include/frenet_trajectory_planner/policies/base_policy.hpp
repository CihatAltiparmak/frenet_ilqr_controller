#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <vector>
#include <iostream>


namespace frenet_trajectory_planner
{
namespace policies
{

class Policy
{
public:
  Policy() {}
  virtual bool check_if_feasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) = 0;
};

template<typename Parameters>
class BasePolicy : public Policy
{
public:
  BasePolicy(const Parameters & parameters)
  : parameters_(parameters)
  {
  }

  Parameters getParameters() const
  {
    return parameters_;
  }

protected:
  Parameters parameters_;
};

}
}
