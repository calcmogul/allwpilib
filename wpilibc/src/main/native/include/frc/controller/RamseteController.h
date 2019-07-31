/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

#include "frc/geometry/Pose2d.h"

namespace frc {

/**
 * TODO: What this controller is, where it's from, why it's useful, and how to
 * use it.
 *
 * See <https://file.tavsys.net/control/state-space-guide.pdf> section on
 * Ramsete unicycle controller for derivation.
 */
class RamseteController {
 public:
  struct Outputs {
    units::meters_per_second_t linearVelocity = 0_mps;
    units::radians_per_second_t angularVelocity = 0_rad_per_s;
  };

  /**
   * Construct a Ramsete unicycle controller.
   *
   * @param b    Tuning parameter (b > 0) for which larger values make
   *             convergence more aggressive like a proportional term.
   * @param zeta Tuning parameter (0 < zeta < 1) for which larger values provide
   *             more damping in response.
   */
  RamseteController(double b, double zeta);

  /**
   * Returns the next output of the Ramsete controller.
   *
   * The desired pose, linear velocity, and angular velocity should come from a
   * drivetrain trajectory.
   *
   * @param desiredPose            The desired pose.
   * @param desiredLinearVelocity  The desired linear velocity.
   * @param desiredAngularVelocity The desired angular velocity.
   * @param currentPose            The current pose.
   */
  Outputs Calculate(const Pose2d& desiredPose,
                    units::meters_per_second_t desiredLinearVelocity,
                    units::radians_per_second_t desiredAngularVelocity,
                    const Pose2d& currentPose);

 private:
  double m_b;
  double m_zeta;
};

}  // namespace frc
