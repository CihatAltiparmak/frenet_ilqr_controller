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
