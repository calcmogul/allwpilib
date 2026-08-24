// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <span>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/trajectory/HolonomicTrajectory.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/velocity.hpp"

namespace wpi::math {

class HolonomicTrajectoryGenerator {
 public:
  /**
   * Generates a trajectory through the given waypoints satisfying the given
   * constraints.
   *
   * @param initialWaypoint The initial waypoint.
   * @param interiorWaypoints The interior waypoints.
   * @param finalWaypoint The final waypoint.
   * @param maxLinearVelocity The maximum linear velocity.
   * @param maxAngularVelocity The maximum angular velocity.
   * @param maxLinearAcceleration The maximum linear acceleration.
   * @param maxAngularAcceleration The maximum angular acceleration.
   */
  static HolonomicTrajectory Generate(
      const Pose2d& initialWaypoint,
      std::span<const Translation2d> interiorWaypoints,
      const Pose2d& finalWaypoint,
      wpi::units::meters_per_second_t maxLinearVelocity,
      wpi::units::radians_per_second_t maxAngularVelocity,
      wpi::units::meters_per_second_squared_t maxLinearAcceleration,
      wpi::units::radians_per_second_squared_t maxAngularAcceleration);

  /**
   * Generates a trajectory through the given waypoints satisfying the given
   * constraints.
   *
   * @param waypoints The list of waypoints.
   * @param maxLinearVelocity The maximum linear velocity.
   * @param maxAngularVelocity The maximum angular velocity.
   * @param maxLinearAcceleration The maximum linear acceleration.
   * @param maxAngularAcceleration The maximum angular acceleration.
   */
  static HolonomicTrajectory Generate(
      std::span<const Pose2d> waypoints,
      wpi::units::meters_per_second_t maxLinearVelocity,
      wpi::units::radians_per_second_t maxAngularVelocity,
      wpi::units::meters_per_second_squared_t maxLinearAcceleration,
      wpi::units::radians_per_second_squared_t maxAngularAcceleration);
};

}  // namespace wpi::math
