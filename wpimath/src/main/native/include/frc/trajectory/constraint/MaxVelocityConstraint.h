// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "units/math.h"
#include "units/velocity.h"

namespace frc {

/**
 * Represents a constraint that enforces a max velocity. This can be composed
 * with the EllipticalRegionConstraint or RectangularRegionConstraint to enforce
 * a max velocity within a region.
 */
class WPILIB_DLLEXPORT MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * Constructs a new MaxVelocityConstraint.
   *
   * @param maxVelocity The max velocity.
   */
  constexpr explicit MaxVelocityConstraint(
      units::meters_per_second_t maxVelocity)
      : m_maxVelocity{units::math::abs(maxVelocity)} {}

  void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
             const sleipnir::Variable& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const sleipnir::Variable& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) const override {
    problem.SubjectTo(linearVelocity >= -m_maxVelocity.value());
    problem.SubjectTo(linearVelocity <= m_maxVelocity.value());
  }

 private:
  units::meters_per_second_t m_maxVelocity;
};

}  // namespace frc
