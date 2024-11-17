#pragma once

#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <vector>
#include <cmath>

namespace frenet_trajectory_planner
{
namespace policies
{

typedef struct AccelerationLimits
{
  double acceleration_min;
  double acceleration_max;
} AccelerationPolicyParameters;

class AccelerationPolicy : public BasePolicy<AccelerationPolicyParameters>
{
public:
  AccelerationPolicy(
    const AccelerationPolicyParameters & acceleration_policy_parameters,
    const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter);
  bool check_if_feasible(
    const FrenetTrajectory & frenet_trajectory,
    const CartesianTrajectory & cartesian_trajectory) override;

private:
  std::shared_ptr<FrenetFrameConverter> frenet_frame_converter_;
};

AccelerationPolicy::AccelerationPolicy(
  const AccelerationPolicyParameters & acceleration_policy_parameters,
  const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter)
: BasePolicy<AccelerationPolicyParameters>(acceleration_policy_parameters),
  frenet_frame_converter_(frenet_frame_converter)
{

}

bool AccelerationPolicy::check_if_feasible(
  const FrenetTrajectory & frenet_trajectory,
  const CartesianTrajectory & cartesian_trajectory)
{

  // assert that frenet_trajectory equals a presentation in frenet frame of cartesian_trajectory
  for (const auto & state : cartesian_trajectory) {
    double acceleration = (state({2, 5})).norm();
    if (acceleration > this->parameters_.acceleration_max ||
      acceleration < this->parameters_.acceleration_min)
    {
      return false;
    }
  }

  return true;
}

}
}
