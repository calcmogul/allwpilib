// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>
#include <wpi/SymbolExports.h>

#include "frc/geometry/Pose2d.h"

namespace frc {

/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
class WPILIB_DLLEXPORT TrajectoryConstraint {
 public:
  TrajectoryConstraint() = default;

  TrajectoryConstraint(const TrajectoryConstraint&) = default;
  TrajectoryConstraint& operator=(const TrajectoryConstraint&) = default;

  TrajectoryConstraint(TrajectoryConstraint&&) = default;
  TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

  virtual ~TrajectoryConstraint() = default;

  /**
   * Apply linear and angular velocity constraints.
   *
   * @param problem The optimization problem.
   * @param pose The robot's pose.
   * @param linearVelocity The robot's linear velocity.
   * @param angularVelocity The robot's angular velocity.
   * @param linearAcceleration The robot's linear acceleration.
   * @param angularAcceleration The robot's angular acceleration.
   */
  virtual void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
                     const sleipnir::Variable& linearVelocity,
                     const sleipnir::Variable& angularVelocity,
                     const sleipnir::Variable& linearAcceleration,
                     const sleipnir::Variable& angularAcceleration) const = 0;
};

}  // namespace frc
