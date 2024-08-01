// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <sleipnir/autodiff/variable.hpp>
#include <sleipnir/optimization/problem.hpp>
#include <wpi/SymbolExports.h>

#include "frc/geometry/Pose2d.h"

namespace frc {

/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
class WPILIB_DLLEXPORT TrajectoryConstraint {
 public:
  constexpr TrajectoryConstraint() = default;

  constexpr TrajectoryConstraint(const TrajectoryConstraint&) = default;
  constexpr TrajectoryConstraint& operator=(const TrajectoryConstraint&) =
      default;

  constexpr TrajectoryConstraint(TrajectoryConstraint&&) = default;
  constexpr TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

  constexpr virtual ~TrajectoryConstraint() = default;

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
  virtual void Apply(slp::Problem& problem, const Pose2d& pose,
                     const slp::Variable& linearVelocity,
                     const slp::Variable& angularVelocity,
                     const slp::Variable& linearAcceleration,
                     const slp::Variable& angularAcceleration) const = 0;
};

}  // namespace frc
