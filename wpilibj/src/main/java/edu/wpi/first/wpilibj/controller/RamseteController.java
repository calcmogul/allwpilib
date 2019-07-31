/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * TODO: What this controller is, where it's from, why it's useful, and how to
 * use it.
 *
 * <p>See <a href="https://file.tavsys.net/control/state-space-guide.pdf">
 * Controls Engineering in the FIRST Robotics Competition</a>
 * section on Ramsete unicycle controller for derivation.
 */
public class RamseteController {
  @SuppressWarnings("MemberName")
  private final double m_b;
  @SuppressWarnings("MemberName")
  private final double m_zeta;

  @SuppressWarnings("MemberName")
  public class Outputs {
    public double linearVelocity;
    public double angularVelocity;

    public Outputs(double linearVelocity, double angularVelocity) {
      this.linearVelocity = linearVelocity;
      this.angularVelocity = angularVelocity;
    }
  }

  /**
   * Construct a Ramsete unicycle controller.
   *
   * @param b    Tuning parameter (b &gt; 0) for which larger values make convergence more
   *             aggressive like a proportional term.
   * @param zeta Tuning parameter (0 &lt; zeta &lt; 1) for which larger values provide more damping
   *             in response.
   */
  @SuppressWarnings("ParameterName")
  public RamseteController(double b, double zeta) {
    m_b = b;
    m_zeta = zeta;
  }

  /**
   * Returns the next output of the Ramsete controller.
   *
   * <p>The desired pose, linear velocity, and angular velocity should come from a drivetrain
   * trajectory.
   *
   * @param desiredPose            The desired pose.
   * @param desiredLinearVelocity  The desired linear velocity.
   * @param desiredAngularVelocity The desired angular velocity.
   * @param currentPose            The current pose.
   */
  @SuppressWarnings("LocalVariableName")
  public Outputs calculate(Pose2d desiredPose,
                           double desiredLinearVelocity,
                           double desiredAngularVelocity,
                           Pose2d currentPose) {
    var error = desiredPose.relativeTo(currentPose);

    // Aliases for equation readability
    double vDesired = desiredLinearVelocity;
    double omegaDesired = desiredAngularVelocity;
    double eX = error.getTranslation().getX();
    double eY = error.getTranslation().getY();
    double eTheta = error.getRotation().getRadians();

    double k = 2.0 * m_zeta
        * Math.sqrt(Math.pow(omegaDesired, 2) + m_b * Math.pow(vDesired, 2));

    return new Outputs(vDesired * Math.cos(eTheta) + k * eX,
                       omegaDesired + k * eTheta + m_b * vDesired * sinc(eTheta) * eY);
  }

  /**
   * Returns sin(x) / x.
   *
   * @param x Value of which to take sinc(x).
   */
  @SuppressWarnings("ParameterName")
  private static double sinc(double x) {
    if (Math.abs(x) < 1e-9) {
      return 1.0;
    } else {
      return Math.sin(x) / x;
    }
  }
}
