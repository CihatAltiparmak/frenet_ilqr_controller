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
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>

#include <vector>
#include <gtest/gtest.h>

TEST(frenet_trajectory_planner, acceleration_policy_test_initialization) {

  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;

  CartesianPoint start_point;
  start_point << 0, 0;
  CartesianPoint final_point;
  final_point << 0, 0;

  AccelerationPolicy<LineAdapter> acceleration_policy(AccelerationPolicyParameters{}, start_point,
    final_point);
}

TEST(frenet_trajectory_planner, acceleration_policy_test_eliminate_frenet_trajectories) {
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using frenet_trajectory_planner::FrenetTrajectory;
  using frenet_trajectory_planner::FrenetState;
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;

  AccelerationPolicyParameters parameters = {
    -1.0, // acceleration_min
    1.0   // acceleration_max
  };

  CartesianPoint start_point;
  start_point << 0, 0;
  CartesianPoint final_point;
  final_point << 1, 0;
  AccelerationPolicy<LineAdapter> acceleration_policy(parameters, start_point, final_point);

  std::vector<FrenetTrajectory> test_trajectory_array;

  {
    FrenetTrajectory trajectory;
    FrenetState state1;
    state1 << 0, 0, 0.5, 0, 0, 0;
    trajectory.push_back(state1);

    test_trajectory_array.push_back(trajectory);
  }

  {
    FrenetTrajectory trajectory;
    FrenetState state1;
    state1 << 0, 0, 1.5, 0, 0, 0.7;
    trajectory.push_back(state1);

    test_trajectory_array.push_back(trajectory);
  }

  auto result_trajectory_array = acceleration_policy.eliminate_frenet_trajectories(
    test_trajectory_array);

  ASSERT_EQ(result_trajectory_array.size(), 1);
}

TEST(frenet_trajectory_planner, acceleration_policy_test_eliminate_cartesian_trajectories) {
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using frenet_trajectory_planner::CartesianTrajectory;
  using frenet_trajectory_planner::CartesianState;
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;

  AccelerationPolicyParameters parameters = {
    -1.0, // acceleration_min
    1.0   // acceleration_max
  };

  CartesianPoint start_point;
  start_point << 0, 0;
  CartesianPoint final_point;
  final_point << 1, 0;
  AccelerationPolicy<LineAdapter> acceleration_policy(parameters, start_point, final_point);

  std::vector<CartesianTrajectory> test_trajectory_array;

  {
    CartesianTrajectory trajectory;
    CartesianState state1;
    state1 << 0, 0, 0.5, 0, 0, 0, 0;
    trajectory.push_back(state1);

    test_trajectory_array.push_back(trajectory);
  }

  {
    CartesianTrajectory trajectory;
    CartesianState state1;
    state1 << 0, 0, 1.5, 0, 0, 0.7, 0;
    trajectory.push_back(state1);

    test_trajectory_array.push_back(trajectory);
  }

  auto result_trajectory_array = acceleration_policy.eliminate_cartesian_trajectories(
    test_trajectory_array);

  ASSERT_EQ(result_trajectory_array.size(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
