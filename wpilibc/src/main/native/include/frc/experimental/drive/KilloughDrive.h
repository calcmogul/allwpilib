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

#include "frc/drive/Vector2d.h"

namespace frc {
namespace experimental {

/**
 * A class for driving Killough drive platforms.
 *
 * Killough drives are triangular with one omni wheel on each corner.
 *
 * Drive base diagram:
 * <pre>
 *  /_____\
 * / \   / \
 *    \ /
 *    ---
 * </pre>
 *
 * Each Drive() function provides different inverse kinematic relations for a
 * Killough drive. The default wheel vectors are parallel to their respective
 * opposite sides, but can be overridden. See the constructor for more
 * information.
 *
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points right, and the
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 */
class KilloughDrive {
 public:
  static constexpr double kDefaultLeftMotorAngle = 60.0;
  static constexpr double kDefaultRightMotorAngle = 120.0;
  static constexpr double kDefaultBackMotorAngle = 270.0;

  class WheelSpeeds {
   public:
    double left = 0.0;
    double right = 0.0;
    double back = 0.0;

    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    void Normalize(double maxSpeed) {
      std::array<double, 3> speedArray{{left, right, back}};
      for (auto& speed : speedArray) {
        speed = std::abs(speed);
      }
      double maxMagnitude =
          *std::max_element(speedArray.begin(), speedArray.end());
      if (maxMagnitude > maxSpeed) {
        left *= maxSpeed / maxMagnitude;
        right *= maxSpeed / maxMagnitude;
        back *= maxSpeed / maxMagnitude;
      }
    }
  };

  KilloughDrive() = default;

  /**
   * Construct a Killough drive with the given motors.
   *
   * Angles are measured in degrees clockwise from the positive X axis.
   *
   * @param leftMotorAngle  The angle of the left wheel's forward direction of
   *                        travel.
   * @param rightMotorAngle The angle of the right wheel's forward direction of
   *                        travel.
   * @param backMotorAngle  The angle of the back wheel's forward direction of
   *                        travel.
   */
  KilloughDrive(double leftMotorAngle, double rightMotorAngle,
                double backMotorAngle);

  KilloughDrive(KilloughDrive&&) = default;
  KilloughDrive& operator=(KilloughDrive&&) = default;

  /**
   * Drive method for Killough platform.
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
   * Drive method for Killough platform.
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

 private:
  frc::Vector2d m_leftVec;
  frc::Vector2d m_rightVec;
  frc::Vector2d m_backVec;
};

}  // namespace experimental
}  // namespace frc
