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

#include <frenet_trajectory_planner/conversion_adapters/circle_adapter.hpp>
#include <frenet_trajectory_planner/type_definitions.hpp>

#include <vector>
#include <gtest/gtest.h>

#define _USE_MATH_DEFINES

TEST(frenet_trajectory_planner, conversion_adapters_circle_adapter_test_initialization) {
  using frenet_trajectory_planner::CartesianPoint;

  CartesianPoint x_start;
  x_start << 1, 1;
  CartesianPoint x_finish;
  x_finish << 1, 2;

  Vector2d c = Vector2d::Zero();
  double r = 1;
  Vector2d x;
  x << -1, 0;
  Vector2d y;
  y << 0, 1;
  double alpha = M_PI / 2;
  [[maybe_unused]] auto adapter = frenet_trajectory_planner::CircleAdapter(
    x_start, x_finish, c, r,
    x, y, alpha);
}

TEST(frenet_trajectory_planner, conversion_adapters_circle_adapter_test_convertFrenet2Cartesian) {
  using frenet_trajectory_planner::CartesianPoint;
  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1, 2;

    Vector2d c = Vector2d::Zero();
    double r = 1;
    Vector2d x;
    x << -1, 0;
    Vector2d y;
    y << 0, 1;
    double alpha = M_PI / 2;

    [[maybe_unused]] auto adapter = frenet_trajectory_planner::CircleAdapter(
      x_start, x_finish, c, r,
      x, y, alpha);

    frenet_trajectory_planner::FrenetState frenet_state =
      frenet_trajectory_planner::FrenetState::Zero();

    // frenet_state[1] = 1;
    // frenet_state[4] = 1;
    // frenet_state[5] = 1;
    // --------------
    // frenet_state[1] = 1;
    // frenet_state[5] = 1;
    // --------------
    // frenet_state[0] = r * (M_PI / 4);
    // frenet_state[3] = 3;
    // --------------
    frenet_state[0] = r * (M_PI / 4);
    frenet_state[1] = 3;
    frenet_state[3] = 3;

    auto cartesian_state = adapter.convertFrenet2Cartesian(frenet_state);
    auto frenet_state2 = adapter.convertCartesian2Frenet(cartesian_state);
    ASSERT_NEAR(cartesian_state[3], M_PI / 4, 1e-10)
      << "This is actual cartesian state: \n"
      << cartesian_state
      << "\nfrenetstate\n"
      << frenet_state
      << "\nconverted frenet state\n"
      << frenet_state2;

  }

  // {
  //   CartesianPoint x_start;
  //   x_start << 1, 1;
  //   CartesianPoint x_finish;
  //   x_finish << 1 + 1 / std::sqrt(2), 1 + 1 / std::sqrt(2);
  //   auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

  //   frenet_trajectory_planner::FrenetState frenet_state =
  //     frenet_trajectory_planner::FrenetState::Zero();
  //   frenet_trajectory_planner::CartesianState cartesian_state = adapter.convertFrenet2Cartesian(
  //     frenet_state);

  //   ASSERT_NEAR(cartesian_state[6], M_PI / 4, 1e-10);
  // }

  // {
  //   CartesianPoint x_start;
  //   x_start << 1, 1;
  //   CartesianPoint x_finish;
  //   x_finish << 2, 1;
  //   auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

  //   frenet_trajectory_planner::FrenetState frenet_state =
  //     frenet_trajectory_planner::FrenetState::Zero();
  //   frenet_trajectory_planner::CartesianState cartesian_state = adapter.convertFrenet2Cartesian(
  //     frenet_state);

  //   ASSERT_NEAR(cartesian_state[6], 0, 1e-10);
  // }
}

// TEST(frenet_trajectory_planner, conversion_adapters_line_adapter_test_convertCartesian2Frenet) {
//   using frenet_trajectory_planner::CartesianPoint;
//   {
//     CartesianPoint x_start;
//     x_start << 1, 1;
//     CartesianPoint x_finish;
//     x_finish << 1, 2;
//     auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

//     frenet_trajectory_planner::CartesianState cartesian_state;
//     cartesian_state << -1, 0, 0, 1, 0, 0, M_PI / 2;

//     frenet_trajectory_planner::FrenetState frenet_state = adapter.convertCartesian2Frenet(
//       cartesian_state);

//     ASSERT_NEAR(frenet_state[3], 2, 1e-10);
//   }

//   {
//     CartesianPoint x_start;
//     x_start << 1, 1;
//     CartesianPoint x_finish;
//     x_finish << 1, 2;
//     auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

//     frenet_trajectory_planner::CartesianState cartesian_state;
//     cartesian_state << -1, 0, 0, 2, 0, 0, M_PI / 2;

//     frenet_trajectory_planner::FrenetState frenet_state = adapter.convertCartesian2Frenet(
//       cartesian_state);

//     ASSERT_NEAR(frenet_state[3], 2, 1e-10);
//     ASSERT_NEAR(frenet_state[0], 1, 1e-10);
//   }

//   {
//     CartesianPoint x_start;
//     x_start << 1, 1;
//     CartesianPoint x_finish;
//     x_finish << 1 + 1 / std::sqrt(2), 1 + 1 / std::sqrt(2);
//     auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

//     frenet_trajectory_planner::CartesianState cartesian_state;
//     cartesian_state << -1, 0, 0, 3, 0, 0, M_PI / 2;

//     frenet_trajectory_planner::FrenetState frenet_state = adapter.convertCartesian2Frenet(
//       cartesian_state);

//     ASSERT_NEAR(frenet_state[3], 2 * std::sqrt(2), 1e-10);
//     ASSERT_NEAR(frenet_state[0], 0, 1e-10);
//   }
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
