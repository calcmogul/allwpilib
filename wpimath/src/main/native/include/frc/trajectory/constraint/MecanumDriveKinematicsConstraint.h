// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/kinematics/MecanumDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {
/**
 * A class that enforces constraints on the mecanum drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for wheels of the drivetrain stay below a certain
 * limit.
 */
class WPILIB_DLLEXPORT MecanumDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  MecanumDriveKinematicsConstraint(const MecanumDriveKinematics& kinematics,
                                   units::meters_per_second_t maxSpeed)
      : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

  void Apply(slp::Problem& problem, const Pose2d& pose,
             const slp::Variable& linearVelocity,
             const slp::Variable& angularVelocity,
             const slp::Variable& linearAcceleration,
             const slp::Variable& angularAcceleration) const override {
    // TODO
    problem.subject_to(linearVelocity > -m_maxSpeed.value());
    problem.subject_to(linearVelocity < m_maxSpeed.value());
  }

 private:
  MecanumDriveKinematics m_kinematics;
  units::meters_per_second_t m_maxSpeed;
};
}  // namespace frc
