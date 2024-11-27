#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>

#include <gtest/gtest.h>

TEST(frenet_trajectory_planner, quartic_trajectory_planner_test_initialization) {
  auto quartic_trajectory_planner = frenet_trajectory_planner::QuarticTrajectoryPlanner();
}

TEST(frenet_trajectory_planner, quartic_trajectory_planner_test_setCoefficientsOrReturnFalse) {
  auto quartic_trajectory_planner = frenet_trajectory_planner::QuarticTrajectoryPlanner();

  ASSERT_EQ(
    quartic_trajectory_planner.setCoefficientsOrReturnFalse(
      0, 0, 0, 1, 0, 0, 1), true);

  // check if the longtitutal velocity is 1 when time is 1 sec
  ASSERT_NEAR(quartic_trajectory_planner.dx(1), 1, 1e-4);
  // check if the longtitutal acceleration is 0 when time is 1 sec
  ASSERT_NEAR(quartic_trajectory_planner.ddx(1), 0, 1e-4);

  // check if the longtitutal distance is 0 when time is 0 sec
  ASSERT_NEAR(quartic_trajectory_planner.x(0), 0, 1e-4);
  // check if the longtitutal velocity is 0 when time is 0 sec
  ASSERT_NEAR(quartic_trajectory_planner.dx(0), 0, 1e-4);
  // check if the longtitutal acceleration is 0 when time is 0 sec
  ASSERT_NEAR(quartic_trajectory_planner.ddx(0), 0, 1e-4);

  // ASSERT_NEAR(quartic_trajectory_planner.x(0.5), 0.5, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
