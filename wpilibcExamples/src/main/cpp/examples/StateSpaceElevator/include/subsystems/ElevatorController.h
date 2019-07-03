/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>
#include <frc/controller/StateSpaceLoop.h>

#include "subsystems/ElevatorCoeffs.h"

class ElevatorController {
 public:
  // State tolerances in meters and meters/sec respectively.
  static constexpr double kPositionTolerance = 0.05;
  static constexpr double kVelocityTolerance = 2.0;

  ElevatorController();

  ElevatorController(ElevatorController&&) noexcept = default;
  ElevatorController& operator=(ElevatorController&&) noexcept = default;

  void Enable();
  void Disable();

  /**
   * Sets the references.
   *
   * @param position Position of the carriage in meters.
   * @param velocity Velocity of the carriage in meters per second.
   */
  void SetReferences(double position, double velocity);

  bool AtReferences() const;

  /**
   * Sets the current encoder measurement.
   *
   * @param measuredPosition Position of the carriage in meters.
   */
  void SetMeasuredPosition(double measuredPosition);

  /**
   * Returns the control loop calculated voltage.
   */
  double ControllerVoltage() const;

  /**
   * Returns the estimated position.
   */
  double EstimatedPosition() const;

  /**
   * Returns the estimated velocity.
   */
  double EstimatedVelocity() const;

  /**
   * Returns the error between the position reference and the position estimate.
   */
  double PositionError() const;

  /**
   * Returns the error between the velocity reference and the velocity estimate.
   */
  double VelocityError() const;

  /**
   * Executes the control loop for a cycle.
   */
  void Update();

  /**
   * Resets any internal state.
   */
  void Reset();

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> m_Y;

  // The control loop.
  frc::StateSpaceLoop<2, 1, 1> m_loop{MakeElevatorLoop()};

  bool m_atReferences = false;
};
