#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>
#include <gtest/gtest.h>

TEST(frenet_trajectory_planner, quintic_trajectory_planner_test_initialization) {
  auto quintic_trajectory_planner = frenet_trajectory_planner::QuinticTrajectoryPlanner();
}

TEST(frenet_trajectory_planner, quintic_trajectory_planner_test_setCoefficientsOrReturnFalse) {
  auto quintic_trajectory_planner = frenet_trajectory_planner::QuinticTrajectoryPlanner();

  ASSERT_EQ(
    quintic_trajectory_planner.setCoefficientsOrReturnFalse(
      0, 0, 0, 1, 0, 0, 0,
      1), true);

  ASSERT_NEAR(quintic_trajectory_planner.x(1), 1, 1e-12);
  ASSERT_NEAR(quintic_trajectory_planner.dx(1), 0, 1e-12);
  ASSERT_NEAR(quintic_trajectory_planner.ddx(1), 0, 1e-12);

  ASSERT_NEAR(quintic_trajectory_planner.x(0), 0, 1e-12);
  ASSERT_NEAR(quintic_trajectory_planner.dx(0), 0, 1e-12);
  ASSERT_NEAR(quintic_trajectory_planner.ddx(0), 0, 1e-12);

  ASSERT_NEAR(quintic_trajectory_planner.x(0.5), 0.5, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
