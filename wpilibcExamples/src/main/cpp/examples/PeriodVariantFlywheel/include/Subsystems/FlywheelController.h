/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>

#include <Eigen/Core>
#include <frc/controller/PeriodVariantLoop.h>

#include "Subsystems/FlywheelCoeffs.h"

class FlywheelController {
 public:
  // Angular velocity tolerance in radians/sec.
  static constexpr double kTolerance = 10.0;

  FlywheelController();

  FlywheelController(FlywheelController&&) = default;
  FlywheelController& operator=(FlywheelController&&) = default;

  void Enable();
  void Disable();

  /**
   * Sets the velocity reference in radians/sec.
   */
  void SetVelocityReference(double angularVelocity);

  bool AtReference() const;

  /**
   * Sets the current encoder velocity in radians/sec.
   */
  void SetMeasuredVelocity(double angularVelocity);

  /**
   * Returns the control loop calculated voltage.
   */
  double ControllerVoltage() const;

  /**
   * Returns the instantaneous velocity.
   */
  double GetEstimatedVelocity() const;

  /**
   * Returns the error between the angular velocity reference and angular
   * velocity estimate.
   */
  double Error() const;

  /**
   * Executes the control loop for a cycle.
   *
   * @param dt Measured time since the last controller update.
   */
  void Update(std::chrono::nanoseconds dt = std::chrono::milliseconds(5));

  /**
   * Resets any internal state.
   */
  void Reset();

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> m_Y;

  // The control loop.
  frc::PeriodVariantLoop<1, 1, 1> m_loop{MakeFlywheelLoop()};

  bool m_atReference = false;
};
