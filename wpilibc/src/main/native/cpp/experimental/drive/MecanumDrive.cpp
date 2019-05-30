/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/experimental/drive/MecanumDrive.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <hal/HAL.h>

#include "frc/drive/Vector2d.h"

using namespace frc::experimental;

constexpr double kPi = 3.14159265358979323846;

MecanumDrive::WheelSpeeds MecanumDrive::DriveCartesian(double xSpeed,
                                                       double ySpeed,
                                                       double zRotation,
                                                       double gyroAngle) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
               HALUsageReporting::kRobotDrive2_MecanumCartesian);
    reported = true;
  }

  // Compensate for gyro angle
  Vector2d input{xSpeed, ySpeed};
  input.Rotate(gyroAngle);

  WheelSpeeds speeds;
  speeds.frontLeft = input.x + input.y + zRotation;
  speeds.frontRight = input.x - input.y - zRotation;
  speeds.rearLeft = input.x - input.y + zRotation;
  speeds.rearRight = input.x + input.y - zRotation;

  return speeds;
}

MecanumDrive::WheelSpeeds MecanumDrive::DrivePolar(double magnitude,
                                                   double angle,
                                                   double zRotation) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
               HALUsageReporting::kRobotDrive2_MecanumPolar);
    reported = true;
  }

  // Convert from polar to rectangular coordinates
  double xSpeed = magnitude * std::cos(angle * (kPi / 180.0));
  double ySpeed = magnitude * std::sin(angle * (kPi / 180.0));
  Vector2d input{xSpeed, ySpeed};

  WheelSpeeds speeds;
  speeds.frontLeft = input.x + input.y + zRotation;
  speeds.frontRight = input.x - input.y - zRotation;
  speeds.rearLeft = input.x - input.y + zRotation;
  speeds.rearRight = input.x + input.y - zRotation;

  return speeds;
}
