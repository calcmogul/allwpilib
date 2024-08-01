// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {

/**
 * A class that enforces constraints on the swerve drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities of the wheels stay below a certain limit.
 */
template <size_t NumModules>
class SwerveDriveKinematicsConstraint : public TrajectoryConstraint {
 public:
  SwerveDriveKinematicsConstraint(
      const frc::SwerveDriveKinematics<NumModules>& kinematics,
      units::meters_per_second_t maxSpeed)
      : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

  void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
             const sleipnir::Variable& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const sleipnir::Variable& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) const override {
    // TODO
    problem.SubjectTo(linearVelocity > -m_maxSpeed.value());
    problem.SubjectTo(linearVelocity < m_maxSpeed.value());
  }

 private:
  frc::SwerveDriveKinematics<NumModules> m_kinematics;
  units::meters_per_second_t m_maxSpeed;
};

}  // namespace frc
