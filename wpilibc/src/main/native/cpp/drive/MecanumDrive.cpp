// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/drive/MecanumDrive.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
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

MecanumDrive::MecanumDrive(SpeedController& frontLeftMotor,
                           SpeedController& rearLeftMotor,
                           SpeedController& frontRightMotor,
                           SpeedController& rearRightMotor)
    : m_frontLeftMotor(&frontLeftMotor),
      m_rearLeftMotor(&rearLeftMotor),
      m_frontRightMotor(&frontRightMotor),
      m_rearRightMotor(&rearRightMotor) {
  wpi::SendableRegistry::AddChild(this, m_frontLeftMotor);
  wpi::SendableRegistry::AddChild(this, m_rearLeftMotor);
  wpi::SendableRegistry::AddChild(this, m_frontRightMotor);
  wpi::SendableRegistry::AddChild(this, m_rearRightMotor);
  static int instances = 0;
  ++instances;
  wpi::SendableRegistry::AddLW(this, "MecanumDrive", instances);
}

void MecanumDrive::DriveCartesian(double xSpeed, double ySpeed,
                                  double zRotation, Rotation2d gyroAngle) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_MecanumCartesian, 4);
    reported = true;
  }

  xSpeed = ApplyDeadband(xSpeed, m_deadband);
  ySpeed = ApplyDeadband(ySpeed, m_deadband);

  auto [frontLeft, frontRight, rearLeft, rearRight] =
      DriveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

  m_frontLeftMotor->Set(frontLeft * m_maxOutput);
  m_frontRightMotor->Set(frontRight * m_maxOutput);
  m_rearLeftMotor->Set(rearLeft * m_maxOutput);
  m_rearRightMotor->Set(rearRight * m_maxOutput);

  Feed();
}

void MecanumDrive::DrivePolar(double magnitude, Rotation2d angle,
                              double zRotation) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_MecanumPolar, 4);
    reported = true;
  }

  DriveCartesian(magnitude * angle.Cos(), magnitude * angle.Sin(), zRotation,
                 0_rad);
}

void MecanumDrive::StopMotor() {
  m_frontLeftMotor->StopMotor();
  m_frontRightMotor->StopMotor();
  m_rearLeftMotor->StopMotor();
  m_rearRightMotor->StopMotor();
  Feed();
}

MecanumDrive::WheelSpeeds MecanumDrive::DriveCartesianIK(double xSpeed,
                                                         double ySpeed,
                                                         double zRotation,
                                                         Rotation2d gyroAngle) {
  xSpeed = std::clamp(xSpeed, -1.0, 1.0);
  ySpeed = std::clamp(ySpeed, -1.0, 1.0);

  Eigen::Vector3d input;
  input << xSpeed, ySpeed, zRotation;

  // CCW rotation matrix
  Eigen::Matrix<double, 3, 3> R;
  double c = gyroAngle.Cos();
  double s = gyroAngle.Sin();
  // clang-format off
  R <<   c,  -s, 0.0,
         s,   c, 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  // Compensate for gyro angle by undoing CCW rotation. Since the rotation
  // matrix is a unitary matrix, it's inverse is equal to its transpose.
  input = R.transpose() * input;

  // Rows are front-left, front-right, rear-left, and rear-right wheel speeds
  // https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
  // section 12.6.1
  Eigen::Matrix<double, 4, 3> M;
  // clang-format off
  M << 1.0, -1.0, -1.0,
       1.0, 1.0, 1.0,
       1.0, 1.0, -1.0,
       1.0, -1.0, 1.0;
  // clang-format on
  Eigen::Matrix<double, 4, 1> output = M * input;

  // Normalize wheel speeds
  double maxValue = output.lpNorm<Eigen::Infinity>();
  if (maxValue > 1.0) {
    output /= maxValue;
  }

  return {output(0), output(1), output(2), output(3)};
}

std::string MecanumDrive::GetDescription() const {
  return "MecanumDrive";
}

void MecanumDrive::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("MecanumDrive");
  builder.SetActuator(true);
  builder.SetSafeState([=] { StopMotor(); });
  builder.AddDoubleProperty(
      "Front Left Motor Speed", [=] { return m_frontLeftMotor->Get(); },
      [=](double value) { m_frontLeftMotor->Set(value); });
  builder.AddDoubleProperty(
      "Front Right Motor Speed", [=] { return m_frontRightMotor->Get(); },
      [=](double value) { m_frontRightMotor->Set(value); });
  builder.AddDoubleProperty(
      "Rear Left Motor Speed", [=] { return m_rearLeftMotor->Get(); },
      [=](double value) { m_rearLeftMotor->Set(value); });
  builder.AddDoubleProperty(
      "Rear Right Motor Speed", [=] { return m_rearRightMotor->Get(); },
      [=](double value) { m_rearRightMotor->Set(value); });
}
