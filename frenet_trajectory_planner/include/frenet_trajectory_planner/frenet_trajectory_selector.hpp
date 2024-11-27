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
  FrenetTrajectorySelector();
  FrenetTrajectorySelector(const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter);

  void addPolicy(const std::shared_ptr<policies::Policy> & policy);
  void addCost(const std::shared_ptr<costs::Cost> & cost);

  std::optional<FrenetTrajectory> selectBestFrenetTrajectory(
    const std::vector<FrenetTrajectory> & frenet_trajectory);

  void setFrenetFrameConverter(
    const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter);

private:
  std::vector<std::shared_ptr<policies::Policy>> policies_;
  std::vector<std::shared_ptr<costs::Cost>> costs_;
  std::shared_ptr<FrenetFrameConverter> frenet_frame_converter_;
};

FrenetTrajectorySelector::FrenetTrajectorySelector()
{
}

FrenetTrajectorySelector::FrenetTrajectorySelector(
  const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter)
: frenet_frame_converter_(frenet_frame_converter)
{

}

void FrenetTrajectorySelector::addPolicy(const std::shared_ptr<policies::Policy> & policy)
{
  policies_.push_back(policy);
}

void FrenetTrajectorySelector::addCost(const std::shared_ptr<costs::Cost> & cost)
{
  costs_.push_back(cost);
}

std::optional<FrenetTrajectory> FrenetTrajectorySelector::selectBestFrenetTrajectory(
  const std::vector<FrenetTrajectory> & frenet_trajectories)
{

  auto policy_checker =
    [this](const FrenetTrajectory & frenet_trajectory,
      const CartesianTrajectory & cartesian_trajectory) -> bool {
      for (const auto & policy : policies_) {
        if (!policy->checkIfFeasible(frenet_trajectory, cartesian_trajectory)) {
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
    CartesianTrajectory cartesian_trajectory = frenet_frame_converter_->convertFrenet2Cartesian(
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

void FrenetTrajectorySelector::setFrenetFrameConverter(
  const std::shared_ptr<FrenetFrameConverter> & frenet_frame_converter)
{
  frenet_frame_converter_ = frenet_frame_converter;
}

}
