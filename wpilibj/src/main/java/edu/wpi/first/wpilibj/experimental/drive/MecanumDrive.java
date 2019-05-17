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
 * A class for driving Mecanum drive platforms.
 *
 * <p>Mecanum drives are rectangular with one wheel on each corner. Each wheel has rollers toed in
 * 45 degrees toward the front or back. When looking at the wheels from the top, the roller axles
 * should form an X across the robot. Each drive() function provides different inverse kinematic
 * relations for a Mecanum drive robot.
 *
 * <p>Drive base diagram:
 * <pre>
 * \\_______/
 * \\ |   | /
 *   |   |
 * /_|___|_\\
 * /       \\
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a Mecanum drive
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
public class MecanumDrive {
  @SuppressWarnings("MemberName")
  public static class WheelSpeeds {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

    public WheelSpeeds() {
    }

    /**
     * Constructs a WheelSpeeds container.
     *
     * @param frontLeft Front left wheel speed.
     * @param frontRight Front right wheel speed.
     * @param rearLeft Rear left wheel speed.
     * @param rearRight Rear right wheel speed.
     */
    public WheelSpeeds(double frontLeft, double frontRight, double rearLeft, double rearRight) {
      this.frontLeft = frontLeft;
      this.frontRight = frontRight;
      this.rearLeft = rearLeft;
      this.rearRight = rearRight;
    }

    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    public void normalize(double maxSpeed) {
      List<Double> speedArray = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);
      for (Double speed : speedArray) {
        speed = Math.abs(speed);
      }
      double maxMagnitude = Collections.max(speedArray);
      if (maxMagnitude > maxSpeed) {
        frontLeft *= maxSpeed / maxMagnitude;
        frontRight *= maxSpeed / maxMagnitude;
        rearLeft *= maxSpeed / maxMagnitude;
        rearRight *= maxSpeed / maxMagnitude;
      }
    }
  }

  private boolean m_reported;

  public MecanumDrive() {
  }

  /**
   * Drive method for Mecanum platform.
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
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(xSpeed, ySpeed, zRotation, 0.0);
  }

  /**
   * Drive method for Mecanum platform.
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
      HAL.report(tResourceType.kResourceType_RobotDrive, 4,
                 tInstances.kRobotDrive2_MecanumCartesian);
      m_reported = true;
    }

    // Compensate for gyro angle
    Vector2d input = new Vector2d(ySpeed, xSpeed);
    input.rotate(-gyroAngle);

    WheelSpeeds speeds = new WheelSpeeds();
    speeds.frontLeft = input.x + input.y + zRotation;
    speeds.frontRight = -input.x + input.y - zRotation;
    speeds.rearLeft = -input.x + input.y + zRotation;
    speeds.rearRight = input.x + input.y - zRotation;

    return speeds;
  }

  /**
   * Drive method for Mecanum platform.
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
      HAL.report(tResourceType.kResourceType_RobotDrive, 4, tInstances.kRobotDrive2_MecanumPolar);
      m_reported = true;
    }

    // Convert from polar to rectangular coordinates
    double xSpeed = magnitude * Math.sin(angle * (Math.PI / 180.0));
    double ySpeed = magnitude * Math.cos(angle * (Math.PI / 180.0));
    Vector2d input = new Vector2d(ySpeed, xSpeed);

    WheelSpeeds speeds = new WheelSpeeds();
    speeds.frontLeft = input.x + input.y + zRotation;
    speeds.frontRight = -input.x + input.y - zRotation;
    speeds.rearLeft = -input.x + input.y + zRotation;
    speeds.rearRight = input.x + input.y - zRotation;

    return speeds;
  }
}
