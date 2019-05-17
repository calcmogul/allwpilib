/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/experimental/drive/DifferentialDrive.h"

#include <hal/HAL.h>

using namespace frc::experimental;

DifferentialDrive::WheelSpeeds DifferentialDrive::ArcadeDrive(
    double xSpeed, double zRotation) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialArcade);
    reported = true;
  }

  return WheelSpeeds{xSpeed + zRotation, xSpeed - zRotation};
}

DifferentialDrive::WheelSpeeds DifferentialDrive::CurvatureDrive(
    double xSpeed, double zRotation, bool allowTurnInPlace) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialCurvature);
    reported = true;
  }

  WheelSpeeds speeds;

  if (allowTurnInPlace) {
    speeds.left = xSpeed + zRotation;
    speeds.right = xSpeed - zRotation;
  } else {
    speeds.left = xSpeed + std::abs(xSpeed) * zRotation;
    speeds.right = xSpeed - std::abs(xSpeed) * zRotation;
  }

  return speeds;
}

DifferentialDrive::WheelSpeeds DifferentialDrive::TankDrive(double leftSpeed,
                                                            double rightSpeed) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialTank);
    reported = true;
  }

  return WheelSpeeds{leftSpeed, rightSpeed};
}
