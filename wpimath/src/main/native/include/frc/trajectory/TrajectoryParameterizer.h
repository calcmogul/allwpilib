// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <wpi/SymbolExports.h>

#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "units/curvature.h"

namespace frc {

/**
 * Class used to parameterize a trajectory by time.
 */
class WPILIB_DLLEXPORT TrajectoryParameterizer {
 public:
  using PoseWithCurvature = std::pair<Pose2d, units::curvature_t>;

  /**
   * Parameterize the trajectory by time. This is where the velocity profile is
   * generated.
   *
   * The derivation of the algorithm used can be found here:
   * <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
   *
   * @param points Reference to the spline points.
   * @param constraints A vector of various velocity and acceleration
   *     constraints.
   * @param startVelocity The start velocity for the trajectory.
   * @param endVelocity The end velocity for the trajectory.
   * @param maxVelocity The max velocity for the trajectory.
   * @param maxAcceleration The max acceleration for the trajectory.
   * @param reversed Whether the robot should move backwards. Note that the
   *     robot will still move from a -> b -> ... -> z as defined in the
   *     waypoints.
   *
   * @return The trajectory.
   */
  static Trajectory TimeParameterizeTrajectory(
      const std::vector<PoseWithCurvature>& points,
      const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
      units::meters_per_second_t startVelocity,
      units::meters_per_second_t endVelocity,
      units::meters_per_second_t maxVelocity,
      units::meters_per_second_squared_t maxAcceleration, bool reversed);

 private:
  constexpr static double kEpsilon = 1E-6;
};

}  // namespace frc
