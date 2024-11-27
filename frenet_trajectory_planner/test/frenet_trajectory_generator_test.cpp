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
