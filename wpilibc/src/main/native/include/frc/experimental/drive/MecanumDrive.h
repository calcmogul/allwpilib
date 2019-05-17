/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace frc {
namespace experimental {

/**
 * A class for driving Mecanum drive platforms.
 *
 * Mecanum drives are rectangular with one wheel on each corner. Each wheel has
 * rollers toed in 45 degrees toward the front or back. When looking at the
 * wheels from the top, the roller axles should form an X across the robot.
 *
 * Drive base diagram:
 * <pre>
 * \\_______/
 * \\ |   | /
 *   |   |
 * /_|___|_\\
 * /       \\
 * </pre>
 *
 * Each Drive() function provides different inverse kinematic relations for a
 * Mecanum drive robot. Motor outputs for the right side are negated, so motor
 * direction inversion by the user is usually unnecessary.
 *
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points to the right,
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 */
class MecanumDrive {
 public:
  class WheelSpeeds {
   public:
    double frontLeft = 0.0;
    double frontRight = 0.0;
    double rearLeft = 0.0;
    double rearRight = 0.0;

    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    void Normalize(double maxSpeed) {
      std::array<double, 4> speedArray{
          {frontLeft, frontRight, rearLeft, rearRight}};
      for (auto& speed : speedArray) {
        speed = std::abs(speed);
      }
      double maxMagnitude =
          *std::max_element(speedArray.begin(), speedArray.end());
      if (maxMagnitude > maxSpeed) {
        frontLeft *= maxSpeed / maxMagnitude;
        frontRight *= maxSpeed / maxMagnitude;
        rearLeft *= maxSpeed / maxMagnitude;
        rearRight *= maxSpeed / maxMagnitude;
      }
    }
  };

  MecanumDrive() = default;

  MecanumDrive(MecanumDrive&&) = default;
  MecanumDrive& operator=(MecanumDrive&&) = default;

  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured clockwise from the positive X axis. The robot's speed
   * is independent from its angle or rotation rate.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around
   *                  the Z axis. Use this to implement field-oriented controls.
   */
  WheelSpeeds DriveCartesian(double xSpeed, double ySpeed, double zRotation,
                             double gyroAngle = 0.0);

  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured clockwise from the positive X axis. The robot's speed
   * is independent from its angle or rotation rate.
   *
   * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is
   *                  positive.
   * @param angle     The angle around the Z axis at which the robot drives in
   *                  degrees [-180..180].
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   */
  WheelSpeeds DrivePolar(double magnitude, double angle, double zRotation);
};

}  // namespace experimental
}  // namespace frc
