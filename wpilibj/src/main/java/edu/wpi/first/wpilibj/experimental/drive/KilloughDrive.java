/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.experimental.drive;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * A class for driving Killough drive platforms.
 *
 * <p>Killough drives are triangular with one omni wheel on each corner.
 *
 * <p>Drive base diagram:
 * <pre>
 *  /_____\
 * / \   / \
 *    \ /
 *    ---
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a Killough drive. The
 * default wheel vectors are parallel to their respective opposite sides, but can be overridden. See
 * the constructor for more information.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 */
public class KilloughDrive {
  @SuppressWarnings("MemberName")
  public static class WheelSpeeds {
    public double left;
    public double right;
    public double back;

    public WheelSpeeds() {
    }

    /**
     * Constructs a WheelSpeeds container.
     *
     * @param left Left wheel speed.
     * @param right Right wheel speed.
     * @param back Back wheel speed.
     */
    public WheelSpeeds(double left, double right, double back) {
      this.left = left;
      this.right = right;
      this.back = back;
    }

    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    public void normalize(double maxSpeed) {
      List<Double> speedArray = Arrays.asList(left, right, back);
      for (Double speed : speedArray) {
        speed = Math.abs(speed);
      }
      double maxMagnitude = Collections.max(speedArray);
      if (maxMagnitude > maxSpeed) {
        left *= maxSpeed / maxMagnitude;
        right *= maxSpeed / maxMagnitude;
        back *= maxSpeed / maxMagnitude;
      }
    }
  }

  public static double kDefaultLeftMotorAngle = 60.0;
  public static double kDefaultRightMotorAngle = 120.0;
  public static double kDefaultBackMotorAngle = 270.0;

  private final Vector2d m_leftVec;
  private final Vector2d m_rightVec;
  private final Vector2d m_backVec;

  private boolean m_reported;

  /**
   * Construct a Killough drive with the given motors.
   *
   * <p>Angles are measured in degrees clockwise from the positive X axis.
   *
   * @param leftMotorAngle  The angle of the left wheel's forward direction of
   *                        travel.
   * @param rightMotorAngle The angle of the right wheel's forward direction of
   *                        travel.
   * @param backMotorAngle  The angle of the back wheel's forward direction of
   *                        travel.
   */
  public KilloughDrive(double leftMotorAngle, double rightMotorAngle,
                       double backMotorAngle) {
    m_leftVec = new Vector2d(Math.cos(leftMotorAngle * (Math.PI / 180.0)),
                             Math.sin(leftMotorAngle * (Math.PI / 180.0)));
    m_rightVec = new Vector2d(Math.cos(rightMotorAngle * (Math.PI / 180.0)),
                              Math.sin(rightMotorAngle * (Math.PI / 180.0)));
    m_backVec = new Vector2d(Math.cos(backMotorAngle * (Math.PI / 180.0)),
                             Math.sin(backMotorAngle * (Math.PI / 180.0)));
  }

  /**
   * Drive method for Killough platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings("ParameterName")
  public WheelSpeeds driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    return driveCartesian(xSpeed, ySpeed, zRotation, 0.0);
  }

  /**
   * Drive method for Killough platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
   *                  this to implement field-oriented controls.
   */
  @SuppressWarnings("ParameterName")
  public WheelSpeeds driveCartesian(double xSpeed, double ySpeed, double zRotation,
                             double gyroAngle) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 3,
                 tInstances.kRobotDrive2_KilloughCartesian);
      m_reported = true;
    }

    // Compensate for gyro angle
    Vector2d input = new Vector2d(xSpeed, ySpeed);
    input.rotate(gyroAngle);

    WheelSpeeds speeds = new WheelSpeeds();
    speeds.left = input.scalarProject(m_leftVec) + zRotation;
    speeds.right = input.scalarProject(m_rightVec) + zRotation;
    speeds.back = input.scalarProject(m_backVec) + zRotation;

    return speeds;
  }

  /**
   * Drive method for Killough platform.
   *
   * <p>Angles are measured counter-clockwise from straight ahead. The speed at which the robot
   * drives (translation) is independent from its angle or rotation rate.
   *
   * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
   * @param angle     The angle around the Z axis at which the robot drives in degrees [-180..180].
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public WheelSpeeds drivePolar(double magnitude, double angle, double zRotation) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 3,
                 tInstances.kRobotDrive2_KilloughPolar);
      m_reported = true;
    }

    // Convert from polar to rectangular coordinates
    double xSpeed = magnitude * Math.cos(angle * (Math.PI / 180.0));
    double ySpeed = magnitude * Math.sin(angle * (Math.PI / 180.0));
    Vector2d input = new Vector2d(xSpeed, ySpeed);

    WheelSpeeds speeds = new WheelSpeeds();
    speeds.left = input.scalarProject(m_leftVec) + zRotation;
    speeds.right = input.scalarProject(m_rightVec) + zRotation;
    speeds.back = input.scalarProject(m_backVec) + zRotation;

    return speeds;
  }
}
