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

#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

class PolynomialTrajectoryPlanner
{
public:
  PolynomialTrajectoryPlanner() {};

  virtual bool setCoefficientsOrReturnFalse(
    const double x0,
    const double dx0,
    const double ddx0,
    const double x1,
    const double dx1,
    const double ddx1,
    const double t0,
    const double t1) {return false;};

  virtual double x(const double t) {return 0.0;};
  virtual double dx(const double t) {return 0.0;};
  virtual double ddx(const double t) {return 0.0;};

protected:
  VectorXd coff_;
};

}
