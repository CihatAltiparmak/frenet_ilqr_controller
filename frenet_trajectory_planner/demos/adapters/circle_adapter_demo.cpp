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
#include <iostream>

int main()
{
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
    // frenet_state[0] = r * (M_PI / 5);
    // frenet_state[1] = 3.78;
    // frenet_state[3] = 3;
    // frenet_state[4] = 3;
    // --------------
    // frenet_state[0] = r * (M_PI / 4);
    // frenet_state[1] = 3;
    // --------------
    // frenet_state[0] = r * (M_PI / 5);
    // frenet_state[1] = 3.78;
    // frenet_state[2] = 11.79;
    // frenet_state[3] = 3;
    // frenet_state[4] = 3;
    // frenet_state[5] = 6;
    // --------------
    frenet_state[0] = r * (M_PI / 5);
    frenet_state[1] = 3.78;
    frenet_state[2] = 11.79313131;
    frenet_state[3] = 13.13;
    frenet_state[4] = 31.32;
    frenet_state[5] = 15.778954;

    auto cartesian_state = adapter.convertFrenet2Cartesian(frenet_state);
    auto frenet_state2 = adapter.convertCartesian2Frenet(cartesian_state);

    std::cout
      << "This is actual cartesian state: " << std::endl
      << cartesian_state << std::endl;

    std::cout << "frenetstate: " << std::endl
              << frenet_state << std::endl;

    std::cout << "converted frenet state: " << std::endl
              << frenet_state2 << std::endl;

    std::cout << "error: " << std::endl
              << (frenet_state2 - frenet_state).norm() << std::endl;

  }
}
