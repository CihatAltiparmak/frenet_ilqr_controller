// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Tufan Kumbasar, Istanbul Technical University Artificial Intelligence and Intelligent Systems (AI2S) Laboratory
// Copyright (C) 2024 Prof. Behçet Uğur Töreyin
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

#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <gtest/gtest.h>

TEST(frenet_trajectory_planner, frenet_trajectory_generator_test_initialization) {
  using frenet_trajectory_planner::FrenetTrajectoryPlannerConfig;

  FrenetTrajectoryPlannerConfig planner_config;
  planner_config.min_lateral_distance = -1;
  planner_config.max_lateral_distance = 1;
  planner_config.step_lateral_distance = 0.1;
  planner_config.min_longtitutal_velocity = -1;
  planner_config.max_longtitutal_velocity = 1;
  planner_config.step_longtitutal_velocity = 0.1;
  frenet_trajectory_planner::FrenetTrajectoryGenerator frenet_trajectory_generator(planner_config);
}

TEST(
  frenet_trajectory_planner,
  frenet_trajectory_generator_test_getAllPossibleFrenetTrajectories) {

  using frenet_trajectory_planner::FrenetState;
  using frenet_trajectory_planner::FrenetTrajectoryPlannerConfig;

  FrenetTrajectoryPlannerConfig planner_config;
  planner_config.min_lateral_distance = -1;
  planner_config.max_lateral_distance = 1;
  planner_config.step_lateral_distance = 0.1;
  planner_config.min_longtitutal_velocity = -1;
  planner_config.max_longtitutal_velocity = 1;
  planner_config.step_longtitutal_velocity = 0.1;

  frenet_trajectory_planner::FrenetTrajectoryGenerator frenet_trajectory_generator(planner_config);

  FrenetState frenet_state_initial = FrenetState::Zero();
  frenet_state_initial[1] = 1;
  frenet_state_initial[3] = -1;
  auto all_frenet_trajectories = frenet_trajectory_generator.getAllPossibleFrenetTrajectories(
    frenet_state_initial);

  ASSERT_EQ(all_frenet_trajectories.size(), 441u);

  auto first_generated_frenet_trajectory = all_frenet_trajectories[0];
  auto final_state_of_first_trajectory = first_generated_frenet_trajectory.back();
  // check if the final longtitutal velocity is -1
  ASSERT_NEAR(final_state_of_first_trajectory[1], -1, 1e-10);
  // check if the lateral distance is -1
  ASSERT_NEAR(final_state_of_first_trajectory[3], -1, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
