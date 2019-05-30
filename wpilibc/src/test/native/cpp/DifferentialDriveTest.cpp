/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <array>
#include <iostream>

#include "frc/experimental/drive/DifferentialDrive.h"
#include "gtest/gtest.h"

const std::array<double, 9> joystickVals{
    {-1.0, -0.9, -0.5, -0.01, 0.0, 0.01, 0.5, 0.9, 1.0}};

TEST(DifferentialDriveTest, TankDrive) {
  frc::experimental::DifferentialDrive drive;
  for (size_t i = 0; i < joystickVals.size(); ++i) {
    for (size_t j = 0; j < joystickVals.size(); ++j) {
      double left = joystickVals[i];
      double right = joystickVals[j];
      auto speeds = drive.TankDrive(left, right);
      ASSERT_FLOAT_EQ(left, speeds.left);
      ASSERT_FLOAT_EQ(right, speeds.right);
    }
  }
}

TEST(DifferentialDriveTest, ArcadeDrive) {
  frc::experimental::DifferentialDrive drive;
  for (size_t i = 0; i < joystickVals.size(); ++i) {
    for (size_t j = 0; j < joystickVals.size(); ++j) {
      double x = joystickVals[i];
      double z = joystickVals[j];
      auto speeds = drive.ArcadeDrive(x, z);
      speeds.Normalize(1.0);
      static_cast<void>(speeds);
      // ASSERT_FLOAT_EQ(, speeds.left);
      // ASSERT_FLOAT_EQ(, speeds.right);
    }
  }
}

TEST(DifferentialDriveTest, CurvatureDrive) {
  frc::experimental::DifferentialDrive drive;
  // const std::array<double, 9> expectedLeft{{}};
  // const std::array<double, 9> expectedRight{{}};
  for (size_t i = 0; i < joystickVals.size(); ++i) {
    for (size_t j = 0; j < joystickVals.size(); ++j) {
      double x = joystickVals[i];
      double z = joystickVals[j];

      auto speeds = drive.CurvatureDrive(x, z, false);
      speeds.Normalize(1.0);
      std::cout << "joy x=" << x << ", y=" << z << "\n";
      std::cout << "  out left=" << speeds.left << ", right=" << speeds.right
                << "\n";
      // ASSERT_FLOAT_EQ(expectedLeft[], speeds.left);
      // ASSERT_FLOAT_EQ(expectedRight[], speeds.right);

      speeds = drive.CurvatureDrive(x, z, true);
      speeds.Normalize(1.0);
      auto arcadeSpeeds = drive.ArcadeDrive(x, z);
      arcadeSpeeds.Normalize(1.0);
      ASSERT_FLOAT_EQ(arcadeSpeeds.left, speeds.left);
      ASSERT_FLOAT_EQ(arcadeSpeeds.right, speeds.right);
    }
  }
}
