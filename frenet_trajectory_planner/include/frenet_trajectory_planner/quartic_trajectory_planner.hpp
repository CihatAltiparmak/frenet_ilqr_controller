#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

class QuarticTrajectoryPlanner
{
public:
  QuarticTrajectoryPlanner();

  bool setCoefficientsOrReturnFalse(
    const double x0,
    const double dx0,
    const double ddx0,
    const double dx1,
    const double ddx1,
    const double t0,
    const double t1);

  double x(const double t);
  double dx(const double t);
  double ddx(const double t);

private:
  VectorXd coff_;
  double x0_;
};

}
