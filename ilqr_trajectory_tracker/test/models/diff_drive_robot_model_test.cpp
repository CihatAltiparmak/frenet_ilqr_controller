#include <ilqr_trajectory_tracker/models/diff_robot_model.hpp>
#include <gtest/gtest.h>

#define _USE_MATH_DEFINES

TEST(ilqr_trajectory_tracker, diff_drive_robot_model_test_initialization) {

  ilqr_trajectory_tracker::DiffDriveRobotModel diff_drive_robot_model;
}

TEST(ilqr_trajectory_tracker, diff_drive_robot_model_test_applySystemDynamics) {

  ilqr_trajectory_tracker::DiffDriveRobotModel diff_drive_robot_model;

  {
    Vector3d x = Vector3d::Zero();
    Vector2d u;
    u << 1, 0;
    double dt = 0.1;
    auto x_final = diff_drive_robot_model.applySystemDynamics(x, u, dt);

    ASSERT_NEAR(x_final[0], 0.1, 1e-4);
  }

  {
    Vector3d x = Vector3d::Zero();
    Vector2d u;
    u << 0, M_PI;
    double dt = 0.1;
    auto x_final = diff_drive_robot_model.applySystemDynamics(x, u, dt);

    ASSERT_NEAR(x_final[0], 0, 1e-4);
    ASSERT_NEAR(x_final[1], 0, 1e-4);
    ASSERT_NEAR(x_final[2], M_PI * dt, 1e-4);
  }
}

TEST(ilqr_trajectory_tracker, diff_drive_robot_model_test_getStateMatrix) {

  ilqr_trajectory_tracker::DiffDriveRobotModel diff_drive_robot_model;

  {
    Vector3d x = Vector3d::Zero();
    Vector2d u;
    u << 1, 0;
    double dt = 0.1;
    auto state_matrix = diff_drive_robot_model.getStateMatrix(x, u, dt);

    ASSERT_NEAR(state_matrix(1, 2), 0.1, 1e-4)
      << "The state matrix in test case is : \n" << state_matrix;
  }
}

TEST(ilqr_trajectory_tracker, diff_drive_robot_model_test_getControlMatrix) {

  ilqr_trajectory_tracker::DiffDriveRobotModel diff_drive_robot_model;

  {
    Vector3d x = Vector3d::Zero();
    Vector2d u;
    u << 1, 0;
    double dt = 0.1;
    auto control_matrix = diff_drive_robot_model.getControlMatrix(x, u, dt);

    ASSERT_NEAR(control_matrix(0, 0), 0.1, 1e-4)
      << "The control matrix in test case is : \n" << control_matrix;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
