// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Dr. Tufan Kumbasar, ITU AI2S Lab
// Copyright (C) 2024 Prof. Dr. Behçet Uğur Töreyin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <omp.h>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/strategies/strategy.hpp>

namespace frenet_trajectory_planner
{

FrenetTrajectoryGenerator::FrenetTrajectoryGenerator(
  const FrenetTrajectoryPlannerConfig & frenet_planner_config)
: frenet_planner_config_(frenet_planner_config)
{}

std::vector<FrenetTrajectory> FrenetTrajectoryGenerator::getAllPossibleFrenetTrajectories(
  const FrenetState & f_state_initial, size_t max_state_number)
{
  return strategy_->generateFrenetTrajectories(f_state_initial, max_state_number);
}

void FrenetTrajectoryGenerator::setStrategy(
  std::unique_ptr<strategies::Strategy> strategy)
{
  strategy_ = std::move(strategy);
}

}  // namespace frenet_trajectory_planner
