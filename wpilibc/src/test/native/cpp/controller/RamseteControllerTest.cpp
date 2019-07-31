/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>

#include "controller/SampleTrajectory.h"
#include "frc/controller/RamseteController.h"
#include "gtest/gtest.h"

static constexpr double kTolerance = 1 / 12.0;
static constexpr double kAngularTolerance = 2.0 * wpi::math::pi / 180.0;

constexpr double boundRadians(double value) {
  while (value > wpi::math::pi) {
    value -= wpi::math::pi * 2;
  }
  while (value <= -wpi::math::pi) {
    value += wpi::math::pi * 2;
  }
  return value;
}

TEST(RamseteControllerTest, ReachesGoal) {
  frc::RamseteController controller{2.0, 0.7};
  frc::Pose2d robotPose{2.7, 23.0, frc::Rotation2d::FromDegrees(0.0)};

  for (size_t i = 0; i <= frc::trajectory.size() - 1; ++i) {
    auto trajectoryPoint = frc::trajectory.at(i);
    double dt = trajectoryPoint.at(0);

    units::foot_t desiredX{trajectoryPoint[1]};
    units::foot_t desiredY{trajectoryPoint[2]};
    frc::Pose2d desiredPose{units::meter_t{desiredX}.to<double>(),
                            units::meter_t{desiredY}.to<double>(),
                            frc::Rotation2d(trajectoryPoint[3])};
    auto desiredVelocity = trajectoryPoint[4];

    double desiredAngularVelocity;
    if (i == frc::trajectory.size() - 1) {
      desiredAngularVelocity = 0.0;
    } else {
      desiredAngularVelocity =
          boundRadians((frc::trajectory[i + 1][3] - trajectoryPoint[3]) / dt);
    }

    auto [linear, angular] = controller.Calculate(
        desiredPose, units::feet_per_second_t(desiredVelocity),
        units::radians_per_second_t(desiredAngularVelocity), robotPose);

    robotPose = robotPose.Exp({units::unit_cast<double>(linear) * dt, 0,
                               units::unit_cast<double>(angular) * dt});
  }

  units::foot_t expectedX{frc::trajectory[frc::trajectory.size() - 1][1]};
  units::foot_t expectedY{frc::trajectory[frc::trajectory.size() - 1][2]};
  double expectedTheta = frc::trajectory[frc::trajectory.size() - 1][3];
  EXPECT_NEAR(units::meter_t{expectedX}.to<double>(),
              robotPose.Translation().X(), kTolerance);
  EXPECT_NEAR(units::meter_t{expectedY}.to<double>(),
              robotPose.Translation().Y(), kTolerance);
  EXPECT_NEAR(boundRadians(expectedTheta - robotPose.Rotation().Radians()), 0.0,
              kAngularTolerance);
}
