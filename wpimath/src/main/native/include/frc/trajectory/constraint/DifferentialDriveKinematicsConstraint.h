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
  DifferentialDriveKinematicsConstraint(DifferentialDriveKinematics kinematics,
                                        units::meters_per_second_t maxSpeed)
      : m_kinematics{std::move(kinematics)}, m_maxSpeed{maxSpeed} {}

  void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
             const sleipnir::Variable& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const sleipnir::Variable& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) const override {
    auto leftWheelVelocity =
        linearVelocity -
        angularVelocity * m_kinematics.trackWidth.value() / 2.0;
    problem.SubjectTo(leftWheelVelocity >= -m_maxSpeed.value());
    problem.SubjectTo(leftWheelVelocity <= m_maxSpeed.value());

    auto rightWheelVelocity =
        linearVelocity +
        angularVelocity * m_kinematics.trackWidth.value() / 2.0;
    problem.SubjectTo(rightWheelVelocity >= -m_maxSpeed.value());
    problem.SubjectTo(rightWheelVelocity <= m_maxSpeed.value());
  }

 private:
  DifferentialDriveKinematics m_kinematics;
  units::meters_per_second_t m_maxSpeed;
};

}  // namespace frc
