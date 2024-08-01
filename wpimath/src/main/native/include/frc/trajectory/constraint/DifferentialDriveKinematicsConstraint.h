// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include <wpi/SymbolExports.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {

/**
 * A class that enforces constraints on the differential drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for both sides of the drivetrain stay below a certain
 * limit.
 */
class WPILIB_DLLEXPORT DifferentialDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  constexpr DifferentialDriveKinematicsConstraint(
      DifferentialDriveKinematics kinematics,
      units::meters_per_second_t maxSpeed)
      : m_kinematics{std::move(kinematics)}, m_maxSpeed{maxSpeed} {}

  void Apply(slp::Problem& problem, const Pose2d& pose,
             const slp::Variable& linearVelocity,
             const slp::Variable& angularVelocity,
             const slp::Variable& linearAcceleration,
             const slp::Variable& angularAcceleration) const override {
    auto leftWheelVelocity =
        linearVelocity -
        angularVelocity * m_kinematics.trackwidth.value() / 2.0;
    problem.subject_to(leftWheelVelocity >= -m_maxSpeed.value());
    problem.subject_to(leftWheelVelocity <= m_maxSpeed.value());

    auto rightWheelVelocity =
        linearVelocity +
        angularVelocity * m_kinematics.trackwidth.value() / 2.0;
    problem.subject_to(rightWheelVelocity >= -m_maxSpeed.value());
    problem.subject_to(rightWheelVelocity <= m_maxSpeed.value());
  }

 private:
  DifferentialDriveKinematics m_kinematics;
  units::meters_per_second_t m_maxSpeed;
};

}  // namespace frc
