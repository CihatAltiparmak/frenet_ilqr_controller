#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>

namespace frenet_trajectory_planner
{

QuinticTrajectoryPlanner::QuinticTrajectoryPlanner()
{}

bool QuinticTrajectoryPlanner::setCoefficientsOrReturnFalse(
  const double x0,
  const double dx0,
  const double ddx0,
  const double x1,
  const double dx1,
  const double ddx1,
  const double t0,
  const double t1
)
{
  MatrixXd T(6, 6);
  // clang-format off
  T << 1, t0, std::pow(t0, 2), std::pow(t0, 3), std::pow(t0, 4), std::pow(t0, 5),
    0, 1, 2 * t0, 3 * std::pow(t0, 2), 4 * std::pow(t0, 3), 5 * std::pow(t0, 4),
    0, 0, 2, 6 * t0, 12 * std::pow(t0, 2), 20 * std::pow(t0, 3),
    1, t1, std::pow(t1, 2), std::pow(t1, 3), std::pow(t1, 4), std::pow(t1, 5),
    0, 1, 2 * t1, 3 * std::pow(t1, 2), 4 * std::pow(t1, 3), 5 * std::pow(t1, 4),
    0, 0, 2, 6 * t1, 12 * std::pow(t1, 2), 20 * std::pow(t1, 3);
  // clang-format on

  VectorXd q(6);
  q << x0, dx0, ddx0, x1, dx1, ddx1;

  auto linear_solver = T.bdcSvd(ComputeThinU | ComputeThinV);

  if (linear_solver.info() != Success) {
    return false;
  }

  coff_ = linear_solver.solve(q);
  return true;
}

double QuinticTrajectoryPlanner::x(const double t)
{
  // clang-format off
  return coff_[0] +
         coff_[1] * t +
         (coff_[2] * std::pow(t, 2)) +
         (coff_[3] * std::pow(t, 3)) +
         (coff_[4] * std::pow(t, 4)) +
         (coff_[5] * std::pow(t, 5));
  // clang-format on
}

double QuinticTrajectoryPlanner::dx(const double t)
{
  // clang-format off
  return coff_[1] +
         2 * coff_[2] * t +
         3 * (coff_[3] * std::pow(t, 2)) +
         4 * (coff_[4] * std::pow(t, 3)) +
         5 * (coff_[5] * std::pow(t, 4));
  // clang-format on
}

double QuinticTrajectoryPlanner::ddx(const double t)
{
  // clang-format off
  return coff_[1] +
         2 * coff_[2] +
         6 * coff_[3] * t +
         12 * (coff_[4] * std::pow(t, 2)) +
         20 * (coff_[5] * std::pow(t, 3));
  // clang-format on
}

}
