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

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>

#include <gtest/gtest.h>


TEST(
  frenet_trajectory_planner,
  frenet_frame_converter_test_frenet2cartesian_converter_initialization) {
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;
  CartesianPoint start_point = CartesianPoint::Zero();
  CartesianPoint final_point;
  final_point << 0, 1;
  frenet_trajectory_planner::Frenet2CartesianConverter<LineAdapter> frenet2cartesian_converter(
    start_point, final_point);
}

TEST(
  frenet_trajectory_planner,
  frenet_frame_converter_test_frenet2cartesian_converter_convert_trajectory) {
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;
  using frenet_trajectory_planner::FrenetTrajectory;
  using frenet_trajectory_planner::CartesianTrajectory;
  using frenet_trajectory_planner::FrenetState;

  CartesianPoint start_point;
  start_point << 1, 1;
  CartesianPoint final_point;
  final_point << 1, 2;
  frenet_trajectory_planner::Frenet2CartesianConverter<LineAdapter> frenet2cartesian_converter(
    start_point, final_point);

  FrenetTrajectory frenet_trajectory;
  frenet_trajectory.push_back(FrenetState::Zero());

  CartesianTrajectory cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    frenet_trajectory);

  ASSERT_NEAR(cartesian_trajectory[0][0], 1, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
