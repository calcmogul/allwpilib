// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/drive/KilloughDrive.h"

#include <algorithm>
#include <cmath>

#include <hal/FRCUsageReporting.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "frc/SpeedController.h"

using namespace frc;

#if defined(_MSC_VER)
#pragma warning(disable : 4996)  // was declared deprecated
#elif defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

const Rotation2d KilloughDrive::kDefaultLeftMotorAngle{60_deg};
const Rotation2d KilloughDrive::kDefaultRightMotorAngle{120_deg};
const Rotation2d KilloughDrive::kDefaultBackMotorAngle{270_deg};

KilloughDrive::KilloughDrive(SpeedController& leftMotor,
                             SpeedController& rightMotor,
                             SpeedController& backMotor)
    : KilloughDrive(leftMotor, rightMotor, backMotor, kDefaultLeftMotorAngle,
                    kDefaultRightMotorAngle, kDefaultBackMotorAngle) {}

KilloughDrive::KilloughDrive(SpeedController& leftMotor,
                             SpeedController& rightMotor,
                             SpeedController& backMotor,
                             Rotation2d leftMotorAngle,
                             Rotation2d rightMotorAngle,
                             Rotation2d backMotorAngle)
    : m_leftMotor(&leftMotor),
      m_rightMotor(&rightMotor),
      m_backMotor(&backMotor) {
  m_leftVec << leftMotorAngle.Cos(), leftMotorAngle.Sin();
  m_rightVec << rightMotorAngle.Cos(), rightMotorAngle.Sin();
  m_backVec << backMotorAngle.Cos(), backMotorAngle.Sin();
  wpi::SendableRegistry::AddChild(this, m_leftMotor);
  wpi::SendableRegistry::AddChild(this, m_rightMotor);
  wpi::SendableRegistry::AddChild(this, m_backMotor);
  static int instances = 0;
  ++instances;
  wpi::SendableRegistry::AddLW(this, "KilloughDrive", instances);
}

void KilloughDrive::DriveCartesian(double xSpeed, double ySpeed,
                                   double zRotation, Rotation2d gyroAngle) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_KilloughCartesian, 3);
    reported = true;
  }

  xSpeed = ApplyDeadband(xSpeed, m_deadband);
  ySpeed = ApplyDeadband(ySpeed, m_deadband);

  auto [left, right, back] =
      DriveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

  m_leftMotor->Set(left * m_maxOutput);
  m_rightMotor->Set(right * m_maxOutput);
  m_backMotor->Set(back * m_maxOutput);

  Feed();
}

void KilloughDrive::DrivePolar(double magnitude, Rotation2d angle,
                               double zRotation) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_KilloughPolar, 3);
    reported = true;
  }

  DriveCartesian(magnitude * angle.Cos(), magnitude * angle.Sin(), zRotation,
                 0_rad);
}

KilloughDrive::WheelSpeeds KilloughDrive::DriveCartesianIK(
    double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
  xSpeed = std::clamp(xSpeed, -1.0, 1.0);
  ySpeed = std::clamp(ySpeed, -1.0, 1.0);

  Eigen::Vector2d input;
  input << xSpeed, ySpeed;

  // CCW rotation matrix
  Eigen::Matrix<double, 2, 2> R;
  double c = gyroAngle.Cos();
  double s = gyroAngle.Sin();
  // clang-format off
  R << c,  -s,
       s,   c;
  // clang-format on

  // Compensate for gyro angle by undoing CCW rotation. Since the rotation
  // matrix is a unitary matrix, it's inverse is equal to its transpose.
  input = R.transpose() * input;

  // Project input onto wheel vectors. Wheel vectors are already normalized, so
  // only dot product is needed.
  Eigen::Matrix<double, 3, 1> output;
  output(0) = input.dot(m_leftVec) + zRotation;
  output(1) = input.dot(m_rightVec) + zRotation;
  output(2) = input.dot(m_backVec) + zRotation;

  // Normalize wheel speeds
  double maxValue = output.lpNorm<Eigen::Infinity>();
  if (maxValue > 1.0) {
    output /= maxValue;
  }

  return {output(0), output(1), output(2)};
}

void KilloughDrive::StopMotor() {
  m_leftMotor->StopMotor();
  m_rightMotor->StopMotor();
  m_backMotor->StopMotor();
  Feed();
}

std::string KilloughDrive::GetDescription() const {
  return "KilloughDrive";
}

void KilloughDrive::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("KilloughDrive");
  builder.SetActuator(true);
  builder.SetSafeState([=] { StopMotor(); });
  builder.AddDoubleProperty(
      "Left Motor Speed", [=] { return m_leftMotor->Get(); },
      [=](double value) { m_leftMotor->Set(value); });
  builder.AddDoubleProperty(
      "Right Motor Speed", [=] { return m_rightMotor->Get(); },
      [=](double value) { m_rightMotor->Set(value); });
  builder.AddDoubleProperty(
      "Back Motor Speed", [=] { return m_backMotor->Get(); },
      [=](double value) { m_backMotor->Set(value); });
}
