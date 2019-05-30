/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <array>

#include "frc/experimental/drive/MecanumDrive.h"
#include "gtest/gtest.h"

const std::array<double, 9> joystickVals = {
    {-1.0, -0.9, -0.5, -0.01, 0.0, 0.01, 0.5, 0.9, 1.0}};
const std::array<double, 19> gyroVals = {{0, 45, 90, 135, 180, 225, 270, 305,
                                          360, 540, -45, -90, -135, -180, -225,
                                          -270, -305, -360, -540}};

TEST(MecanumDriveTest, DriveCartesian) {
  frc::experimental::MecanumDrive drive;
  for (size_t i = 0; i < joystickVals.size(); ++i) {
    for (size_t j = 0; j < joystickVals.size(); ++j) {
      for (size_t k = 0; k < joystickVals.size(); ++k) {
        for (size_t l = 0; l < gyroVals.size(); ++l) {
          double xJoystick = joystickVals[i];
          double yJoystick = joystickVals[j];
          double rotateJoystick = joystickVals[k];
          double gyroValue = gyroVals[l];
          auto speeds = drive.DriveCartesian(xJoystick, -yJoystick,
                                             rotateJoystick, -gyroValue);
          static_cast<void>(speeds);
          // ASSERT_NEAR(m_rdFrontLeft.Get(), m_frontLeft.Get(), 0.01)
          //    << "X: " << xJoystick << " Y: " << yJoystick
          //    << " Rotate: " << rotateJoystick << " Gyro: " << gyroValue;
          // ASSERT_NEAR(m_rdFrontRight.Get(), -m_frontRight.Get(), 0.01)
          //    << "X: " << xJoystick << " Y: " << yJoystick
          //    << " Rotate: " << rotateJoystick << " Gyro: " << gyroValue;
          // ASSERT_NEAR(m_rdRearLeft.Get(), m_rearLeft.Get(), 0.01)
          //    << "X: " << xJoystick << " Y: " << yJoystick
          //    << " Rotate: " << rotateJoystick << " Gyro: " << gyroValue;
          // ASSERT_NEAR(m_rdRearRight.Get(), -m_rearRight.Get(), 0.01)
          //    << "X: " << xJoystick << " Y: " << yJoystick
          //    << " Rotate: " << rotateJoystick << " Gyro: " << gyroValue;
        }
      }
    }
  }
}

TEST(MecanumDriveTest, DrivePolar) {
  frc::experimental::MecanumDrive drive;
  for (size_t i = 0; i < joystickVals.size(); ++i) {
    for (size_t j = 0; j < joystickVals.size(); ++j) {
      for (size_t k = 0; k < gyroVals.size(); ++k) {
        double magnitudeJoystick = joystickVals[i];
        double rotateJoystick = joystickVals[j];
        double directionJoystick = gyroVals[k];
        auto speeds = drive.DrivePolar(magnitudeJoystick, directionJoystick,
                                       rotateJoystick);
        static_cast<void>(speeds);
        // ASSERT_NEAR(m_rdFrontLeft.Get(), m_frontLeft.Get(), 0.01)
        //    << "Magnitude: " << magnitudeJoystick
        //    << " Direction: " << directionJoystick
        //    << " Rotate: " << rotateJoystick;
        // ASSERT_NEAR(m_rdFrontRight.Get(), -m_frontRight.Get(), 0.01)
        //    << "Magnitude: " << magnitudeJoystick
        //    << " Direction: " << directionJoystick
        //    << " Rotate: " << rotateJoystick;
        // ASSERT_NEAR(m_rdRearLeft.Get(), m_rearLeft.Get(), 0.01)
        //    << "Magnitude: " << magnitudeJoystick
        //    << " Direction: " << directionJoystick
        //    << " Rotate: " << rotateJoystick;
        // ASSERT_NEAR(m_rdRearRight.Get(), -m_rearRight.Get(), 0.01)
        //    << "Magnitude: " << magnitudeJoystick
        //    << " Direction: " << directionJoystick
        //    << " Rotate: " << rotateJoystick;
      }
    }
  }
}
