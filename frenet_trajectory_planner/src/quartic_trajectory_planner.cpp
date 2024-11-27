#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>

namespace frenet_trajectory_planner
{

QuarticTrajectoryPlanner::QuarticTrajectoryPlanner()
{}

bool QuarticTrajectoryPlanner::setCoefficientsOrReturnFalse(
  const double x0,
  const double dx0,
  const double ddx0,
  const double dx1,
  const double ddx1,
  const double t0,
  const double t1
)
{
  MatrixXd T(4, 4);
  // clang-format off
  T <<
    1, 2 * t0, 3 * std::pow(t0, 2), 4 * std::pow(t0, 3),
    0, 2, 6 * t0, 12 * std::pow(t0, 2),
    1, 2 * t1, 3 * std::pow(t1, 2), 4 * std::pow(t1, 3),
    0, 2, 6 * t1, 12 * std::pow(t1, 2);
  // clang-format on

  VectorXd q(4);
  q << dx0, ddx0, dx1, ddx1;

  auto linear_solver = T.bdcSvd(ComputeThinU | ComputeThinV);

  if (linear_solver.info() != Success) {
    return false;
  }

  coff_ = linear_solver.solve(q);
  x0_ = x0;
  return true;
}

double QuarticTrajectoryPlanner::x(const double t)
{
  // clang-format off
  return x0_ +
         coff_[0] * t +
         (coff_[1] * std::pow(t, 2)) +
         (coff_[2] * std::pow(t, 3)) +
         (coff_[3] * std::pow(t, 4));
  // clang-format on
}

double QuarticTrajectoryPlanner::dx(const double t)
{
  // clang-format off
  return coff_[0] +
         2 * coff_[1] * t +
         3 * (coff_[2] * std::pow(t, 2)) +
         4 * (coff_[3] * std::pow(t, 3));
  // clang-format on
}

double QuarticTrajectoryPlanner::ddx(const double t)
{
  // clang-format off
  return 2 * coff_[1] +
         6 * coff_[2] * t +
         12 * (coff_[3] * std::pow(t, 2));
  // clang-format on
}

}
