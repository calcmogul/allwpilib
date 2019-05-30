/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <algorithm>
#include <cmath>

namespace frc {
namespace experimental {

/**
 * A class for driving differential drive/skid-steer drive platforms such as
 * the Kit of Parts drive base, "tank drive", or West Coast Drive.
 *
 * These drive bases typically have drop-center / skid-steer with two or more
 * wheels per side (e.g., 6WD or 8WD). This class takes a SpeedController per
 * side. For four and six motor drivetrains, construct and pass in
 * SpeedControllerGroup instances as follows.
 *
 * Four motor drivetrain:
 * @code{.cpp}
 * class Robot {
 *  public:
 *   frc::Spark m_frontLeft{1};
 *   frc::Spark m_rearLeft{2};
 *   frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};
 *
 *   frc::Spark m_frontRight{3};
 *   frc::Spark m_rearRight{4};
 *   frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};
 *
 *   frc::DifferentialDrive m_drive;
 *   frc::Joystick m_forwardJoystick{0};
 *   frc::Joystick m_turnJoystick{1};
 *
 *   RobotInit() {
 *     m_forwardJoystick.SetAxisDeadband(0.02);
 *     m_turnJoystick.SetAxisDeadband(0.02);
 *   }
 *
 *   void TeleopPeriodic() {
 *     auto speeds = m_drive.CurvatureDrive(forwardJoystick.GetY(),
 *                                          turnJoystick.GetX(), false);
 *     m_left.Set(speeds.left);
 *     m_right.Set(speeds.right);
 *   }
 * };
 * @endcode
 *
 * Six motor drivetrain:
 * @code{.cpp}
 * class Robot {
 *  public:
 *   frc::Spark m_frontLeft{1};
 *   frc::Spark m_midLeft{2};
 *   frc::Spark m_rearLeft{3};
 *   frc::SpeedControllerGroup m_left{m_frontLeft, m_midLeft, m_rearLeft};
 *
 *   frc::Spark m_frontRight{4};
 *   frc::Spark m_midRight{5};
 *   frc::Spark m_rearRight{6};
 *   frc::SpeedControllerGroup m_right{m_frontRight, m_midRight, m_rearRight};
 *
 *   frc::DifferentialDrive m_drive;
 *   frc::Joystick m_forwardJoystick{0};
 *   frc::Joystick m_turnJoystick{1};
 *
 *   RobotInit() {
 *     m_forwardJoystick.SetAxisDeadband(0.02);
 *     m_turnJoystick.SetAxisDeadband(0.02);
 *   }
 *
 *   void TeleopPeriodic() {
 *     auto speeds = m_drive.CurvatureDrive(forwardJoystick.GetY(),
 *                                          turnJoystick.GetX(), false);
 *     m_left.Set(speeds.left);
 *     m_right.Set(speeds.right);
 *   }
 * };
 * @endcode
 *
 * A differential drive robot has left and right wheels separated by an
 * arbitrary width.
 *
 * Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * Each Drive() function provides different inverse kinematic relations for a
 * differential drive robot. Motor outputs for the right side are negated, so
 * motor direction inversion by the user is usually unnecessary.
 *
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points to the right,
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 */
class DifferentialDrive {
 public:
  class WheelSpeeds {
   public:
    double left = 0.0;
    double right = 0.0;

    /**
     * Normalizes the wheel speeds to a symmetric range with the given
     * magnitude.
     *
     * @param maxSpeed Maximum wheel speed in normalization range
     *                 [-maxSpeed..maxSpeed].
     */
    void Normalize(double maxSpeed) {
      double maxMagnitude = std::max(std::abs(left), std::abs(right));
      if (maxMagnitude > maxSpeed) {
        left *= maxSpeed / maxMagnitude;
        right *= maxSpeed / maxMagnitude;
      }
    }
  };

  DifferentialDrive() = default;

  DifferentialDrive(DifferentialDrive&&) = default;
  DifferentialDrive& operator=(DifferentialDrive&&) = default;

  /**
   * Arcade drive method for differential drive platform.
   *
   * Note: Some drivers may prefer inverted rotation controls. This can be done
   * by negating the value passed for rotation.
   *
   * @param xSpeed        The speed at which the robot should drive along the X
   *                      axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The rotation rate of the robot around the Z axis
   *                      [-1.0..1.0]. Clockwise is positive.
   */
  WheelSpeeds ArcadeDrive(double xSpeed, double zRotation);

  /**
   * Curvature drive method for differential drive platform.
   *
   * The rotation argument controls the curvature of the robot's path rather
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
  WheelSpeeds CurvatureDrive(double xSpeed, double zRotation,
                             bool allowTurnInPlace);

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed     The robot left side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   * @param rightSpeed    The robot right side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   */
  WheelSpeeds TankDrive(double leftSpeed, double rightSpeed);
};

}  // namespace experimental
}  // namespace frc
