/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controller/SampleTrajectory.h"
#include "frc/controller/RamseteController.h"
#include "gtest/gtest.h"

using namespace frc;

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kTolerance = 1 / 12.0;
static constexpr double kAngularTolerance = 2.0 * kPi / 180.0;

constexpr double boundRadians(double value) {
  while (value > kPi) {
    value -= kPi * 2;
  }
  while (value <= -kPi) {
    value += kPi * 2;
  }
  return value;
}

TEST(RamseteControllerTest, FollowerReachesGoal) {
  RamseteController controller{2.0, 0.7};
  int index = 0;

  Pose2d robotPose{2.7, 23.0, Rotation2d::FromDegrees(0.0)};

  while (index <= trajectory.size() - 1) {
    auto trajectoryPoint = trajectory.at(index);
    double dt = trajectoryPoint.at(0);

    auto desiredPose = Pose2d(trajectoryPoint[1] * 12 * 0.0254,
                              trajectoryPoint[2] * 12 * 0.0254,
                              Rotation2d(trajectoryPoint[3]));
    auto desiredVelocity = trajectoryPoint[4];
    auto desiredAngularVelocity =
        index == trajectory.size() - 1
            ? 0.0
            : boundRadians((trajectory[index + 1][3] - trajectoryPoint[3]) /
                           dt);

    auto [linear, angular] = controller.Calculate(
        desiredPose, units::feet_per_second_t(desiredVelocity),
        units::radians_per_second_t(desiredAngularVelocity), robotPose);

    robotPose = robotPose.Exp({units::unit_cast<double>(linear) * dt, 0,
                               units::unit_cast<double>(angular) * dt});

    index++;
  }

  EXPECT_NEAR(trajectory[trajectory.size() - 1][1] * 12 * 0.0254,
              robotPose.Translation().X(), kTolerance);
  EXPECT_NEAR(trajectory[trajectory.size() - 1][2] * 12 * 0.0254,
              robotPose.Translation().Y(), kTolerance);
  EXPECT_NEAR(boundRadians(trajectory[trajectory.size() - 1][3] -
                           robotPose.Rotation().Radians()),
              0.0, kAngularTolerance);
}
