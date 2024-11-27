#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>
#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;

namespace frenet_trajectory_planner
{

class FrenetTrajectoryGenerator
{
public:
  FrenetTrajectoryGenerator(const FrenetTrajectoryPlannerConfig & frenet_planner_config);

  std::vector<FrenetTrajectory> getAllPossibleFrenetTrajectories(
    const FrenetState & frenet_state_initial);

  FrenetTrajectory getFrenetTrajectory(
    const FrenetState & frenet_state_initial,
    const FrenetState & frenet_state_final);

private:
  FrenetTrajectoryPlannerConfig frenet_planner_config_;
};

}
