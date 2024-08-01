// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "units/acceleration.h"

namespace frc {

/**
 * A constraint on the maximum absolute centripetal acceleration allowed when
 * traversing a trajectory. The centripetal acceleration of a robot is defined
 * as the velocity squared divided by the radius of curvature.
 *
 * Effectively, limiting the maximum centripetal acceleration will cause the
 * robot to slow down around tight turns, making it easier to track trajectories
 * with sharp turns.
 */
class WPILIB_DLLEXPORT CentripetalAccelerationConstraint
    : public TrajectoryConstraint {
 public:
  constexpr explicit CentripetalAccelerationConstraint(
      units::meters_per_second_squared_t maxCentripetalAcceleration)
      : m_maxCentripetalAcceleration{maxCentripetalAcceleration} {}

  void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
             const sleipnir::Variable& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const sleipnir::Variable& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) const override {
    // Find max linear velocity for max centripetal acceleration.
    //
    //   a_c = v²/r   (1)
    //   v = rω       (2)
    //
    // Solve (2) for r.
    //
    //   r = v/ω
    //
    // Substitute r into (1).
    //
    //   a_c = v²/r
    //   a_c = v²/(v/ω)
    //   a_c = vω
    //
    // Solve for v.
    //
    //   v = a_c/ω
    //
    // Write out the constraints.
    //
    //   v ≥ -a_c/ω
    //   v ≤ a_c/ω
    //
    //   vω ≥ -a_c
    //   vω ≤ a_c
    problem.SubjectTo(linearVelocity * angularVelocity >=
                      -m_maxCentripetalAcceleration.value());
    problem.SubjectTo(linearVelocity * angularVelocity <=
                      m_maxCentripetalAcceleration.value());
  }

 private:
  units::meters_per_second_squared_t m_maxCentripetalAcceleration;
};

}  // namespace frc
