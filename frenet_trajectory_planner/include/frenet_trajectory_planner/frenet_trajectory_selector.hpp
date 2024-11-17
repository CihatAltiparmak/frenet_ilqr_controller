#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/costs/base_cost.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <vector>
#include <memory>
#include <optional>
#include <limits>

namespace frenet_trajectory_planner
{

class FrenetTrajectorySelector
{
public:
  FrenetTrajectorySelector(const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter);

  void add_policy(const std::shared_ptr<policies::Policy> & policy);
  void add_cost(const std::shared_ptr<costs::Cost> & cost);

  std::optional<FrenetTrajectory> select_best_frenet_trajectory(
    const std::vector<FrenetTrajectory> & frenet_trajectory);

private:
  std::vector<std::shared_ptr<policies::Policy>> policies_;
  std::vector<std::shared_ptr<costs::Cost>> costs_;
  std::shared_ptr<FrenetFrameConverter> frenet_frame_converter_;
};

FrenetTrajectorySelector::FrenetTrajectorySelector(
  const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter)
: frenet_frame_converter_(frenet_frame_converter)
{

}

void FrenetTrajectorySelector::add_policy(const std::shared_ptr<policies::Policy> & policy)
{
  policies_.push_back(policy);
}

void FrenetTrajectorySelector::add_cost(const std::shared_ptr<costs::Cost> & cost)
{
  costs_.push_back(cost);
}

std::optional<FrenetTrajectory> FrenetTrajectorySelector::select_best_frenet_trajectory(
  const std::vector<FrenetTrajectory> & frenet_trajectories)
{

  auto policy_checker =
    [this](const FrenetTrajectory & frenet_trajectory,
      const CartesianTrajectory & cartesian_trajectory) -> bool {
      for (const auto & policy : policies_) {
        if (!policy->check_if_feasible(frenet_trajectory, cartesian_trajectory)) {
          return false;
        }
      }
      return true;
    };

  auto get_trajectory_cost = [this](const FrenetTrajectory & frenet_trajectory) -> double {
      double trajectory_cost = 0;
      for (const auto & cost_checker : costs_) {
        trajectory_cost += cost_checker->cost(frenet_trajectory);
      }
      return trajectory_cost;
    };

  std::optional<FrenetTrajectory> best_frenet_trajectory = std::nullopt;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & frenet_trajectory : frenet_trajectories) {
    CartesianTrajectory cartesian_trajectory = frenet_frame_converter_->convert_frenet2cartesian(
      frenet_trajectory);
    if (!policy_checker(frenet_trajectory, cartesian_trajectory)) {
      continue;
    }

    // check for cost
    double trajectory_cost = get_trajectory_cost(frenet_trajectory);
    if (trajectory_cost < best_cost) {
      best_cost = trajectory_cost;
      best_frenet_trajectory = std::optional<FrenetTrajectory>{frenet_trajectory};
    }
  }

  return best_frenet_trajectory;
}

}
