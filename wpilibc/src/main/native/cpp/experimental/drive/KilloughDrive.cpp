/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/experimental/drive/KilloughDrive.h"

#include <hal/HAL.h>

using namespace frc::experimental;

constexpr double kPi = 3.14159265358979323846;

KilloughDrive::KilloughDrive(double leftMotorAngle, double rightMotorAngle,
                             double backMotorAngle) {
  m_leftVec = {std::cos(leftMotorAngle * (kPi / 180.0)),
               std::sin(leftMotorAngle * (kPi / 180.0))};
  m_rightVec = {std::cos(rightMotorAngle * (kPi / 180.0)),
                std::sin(rightMotorAngle * (kPi / 180.0))};
  m_backVec = {std::cos(backMotorAngle * (kPi / 180.0)),
               std::sin(backMotorAngle * (kPi / 180.0))};
}

KilloughDrive::WheelSpeeds KilloughDrive::DriveCartesian(double xSpeed,
                                                         double ySpeed,
                                                         double zRotation,
                                                         double gyroAngle) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 3,
               HALUsageReporting::kRobotDrive2_KilloughCartesian);
    reported = true;
  }

  // Compensate for gyro angle
  Vector2d input{ySpeed, xSpeed};
  input.Rotate(-gyroAngle);

  WheelSpeeds speeds;
  speeds.left = input.ScalarProject(m_leftVec) + zRotation;
  speeds.right = input.ScalarProject(m_rightVec) + zRotation;
  speeds.back = input.ScalarProject(m_backVec) + zRotation;

  return speeds;
}

KilloughDrive::WheelSpeeds KilloughDrive::DrivePolar(double magnitude,
                                                     double angle,
                                                     double zRotation) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 3,
               HALUsageReporting::kRobotDrive2_KilloughPolar);
    reported = true;
  }

  // Convert from polar to rectangular coordinates
  double xSpeed = magnitude * std::cos(angle * (kPi / 180.0));
  double ySpeed = magnitude * std::sin(angle * (kPi / 180.0));
  Vector2d input{xSpeed, ySpeed};

  WheelSpeeds speeds;
  speeds.left = input.ScalarProject(m_leftVec) + zRotation;
  speeds.right = input.ScalarProject(m_rightVec) + zRotation;
  speeds.back = input.ScalarProject(m_backVec) + zRotation;

  return speeds;
}
