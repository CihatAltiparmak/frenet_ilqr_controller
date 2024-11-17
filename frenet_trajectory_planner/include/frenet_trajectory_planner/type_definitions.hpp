#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

using StateLateral = Vector3d;
using StateLongtitutal = Vector3d;
using FrenetState = Vector<double, 6>; // s, s_dot, s_dot_dot, d, d_dot, d_dot_dot
using FrenetTrajectory = std::vector<FrenetState>;

using CartesianState = Vector<double, 7>; // x, x_dot, x_dot_dot, y, y_dot, y_dot_dot, yaw
using CartesianTrajectory = std::vector<CartesianState>;

using CartesianPoint = Vector2d;

typedef struct FrenetTrajectoryPlannerConfig
{
  double min_lateral_distance;
  double max_lateral_distance;
  double step_lateral_distance;
  double min_longtitutal_velocity;
  double max_longtitutal_velocity;
  double step_longtitutal_velocity;
  double time_interval = 100; // 1; // 100;
  double dt = 0.005; // time discretization
} FrenetTrajectoryPlannerConfig;

}
