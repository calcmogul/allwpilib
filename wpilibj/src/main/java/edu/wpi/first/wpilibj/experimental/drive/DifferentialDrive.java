/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.experimental.drive;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

/**
 * A class for driving differential drive/skid-steer drive platforms such as the Kit of Parts drive
 * base, "tank drive", or West Coast Drive.
 *
 * <p>These drive bases typically have drop-center / skid-steer with two or more wheels per side
 * (e.g., 6WD or 8WD). This class takes a SpeedController per side. For four and
 * six motor drivetrains, construct and pass in {@link edu.wpi.first.wpilibj.SpeedControllerGroup}
 * instances as follows.
 *
 * <p>Four motor drivetrain:
 * <pre><code>
 * public class Robot {
 *   Spark m_frontLeft = new Spark(1);
 *   Spark m_rearLeft = new Spark(2);
 *   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
 *
 *   Spark m_frontRight = new Spark(3);
 *   Spark m_rearRight = new Spark(4);
 *   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
 *
 *   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 * }
 * </code></pre>
 *
 * <p>Six motor drivetrain:
 * <pre><code>
 * public class Robot {
 *   Spark m_frontLeft = new Spark(1);
 *   Spark m_midLeft = new Spark(2);
 *   Spark m_rearLeft = new Spark(3);
 *   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_midLeft, m_rearLeft);
 *
 *   Spark m_frontRight = new Spark(4);
 *   Spark m_midRight = new Spark(5);
 *   Spark m_rearRight = new Spark(6);
 *   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_midRight, m_rearRight);
 *
 *   DifferentialDrive m_drive = new DifferentialDrive();
 *   Joystick m_forwardJoystick = new Joystick(0);
 *   Joystick m_turnJoystick = new Joystick(1);
 *
 *   public robotInit() {
 *     m_forwardJoystick.setAxisDeadband(0.02);
 *     m_turnJoystick.setAxisDeadband(0.02);
 *   }
 *
 *   public void teleopPeriodic() {
 *     DifferentialDrive.WheelSpeeds speeds = m_drive.curvatureDrive(forwardJoystick.getY(),
 *                                                                   turnJoystick.getX(), false);
 *     m_left.Set(speeds.left);
 *     m_right.Set(speeds.right);
 *   }
 * }
 * </code></pre>
 *
 * <p>A differential drive robot has left and right wheels separated by an arbitrary width.
 *
 * <p>Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a differential drive
 * robot. Motor outputs for the right side are negated, so motor direction inversion by the user is
 * usually unnecessary.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 */
public class DifferentialDrive {
  @SuppressWarnings("MemberName")
  public static class WheelSpeeds {
    public double left;
    public double right;

    public WheelSpeeds() {
    }

    /**
     * Constructs a WheelSpeeds container.
     *
     * @param left Left wheel speed.
     * @param right Right wheel speed.
     */
    public WheelSpeeds(double left, double right) {
      this.left = left;
      this.right = right;
    }


    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    public void normalize(double maxSpeed) {
      double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
      if (maxMagnitude > maxSpeed) {
        left *= maxSpeed / maxMagnitude;
        right *= maxSpeed / maxMagnitude;
      }
    }
  }

  private boolean m_reported;

  public DifferentialDrive() {
  }

  /**
   * Arcade drive method for differential drive platform.
   *
   * <p>Note: Some drivers may prefer inverted rotation controls. This can be
   * done by negating the value passed for rotation.
   *
   * @param xSpeed        The speed at which the robot should drive along the X
   *                      axis [-1.0..1.0]. Forward is negative.
   * @param zRotation     The rotation rate of the robot around the Z axis
   *                      [-1.0..1.0]. Clockwise is positive.
   */
  @SuppressWarnings("ParameterName")
  public WheelSpeeds arcadeDrive(double xSpeed, double zRotation) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialArcade);
      m_reported = true;
    }

    return new WheelSpeeds(xSpeed + zRotation, xSpeed - zRotation);
  }

  /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather
   * than its rate of heading change. This makes the robot more controllable at
   * high speeds. Constant-curvature turning can be overridden for turn-in-place
   * maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward
   *                    is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0].
   *                    Clockwise is positive.
   * @param allowTurnInPlace If set, overrides constant-curvature turning for
   *                         turn-in-place maneuvers.
   */
  @SuppressWarnings("ParameterName")
  public WheelSpeeds curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialCurvature);
      m_reported = true;
    }

    WheelSpeeds speeds = new WheelSpeeds();

    if (allowTurnInPlace) {
      speeds.left = xSpeed + zRotation;
      speeds.right = xSpeed - zRotation;
    } else {
      speeds.left = xSpeed + Math.abs(xSpeed) * zRotation;
      speeds.right = xSpeed - Math.abs(xSpeed) * zRotation;
    }

    return speeds;
  }

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed     The robot left side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   * @param rightSpeed    The robot right side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   */
  public WheelSpeeds tankDrive(double leftSpeed, double rightSpeed) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialTank);
      m_reported = true;
    }

    return new WheelSpeeds(leftSpeed, rightSpeed);
  }
}
