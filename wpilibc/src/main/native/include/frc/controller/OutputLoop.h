/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

#include "frc/controller/OutputController.h"

namespace frc {

/**
 * Runs an output controller in feedback.
 *
 * For everything in this file, "inputs" and "outputs" are defined from the
 * perspective of the plant. This means U is an input and Y is an output
 * (because you give the plant U (powers) and it gives you back a Y (sensor
 * values). This is the opposite of what they mean from the perspective of the
 * controller (U is an output because that's what goes to the motors and Y is an
 * input because that's what comes back from the sensors).
 */
template <int Inputs, int Outputs>
class OutputLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * A convenience constructor for constructing an output loop with the given
   * controller coefficients.
   *
   * @param controllerCoeffs Output controller coefficients.
   */
  explicit OutputLoop(
      const OutputControllerCoeffs<Inputs, Outputs>& controllerCoeffs);

  /**
   * Constructs an output loop with the given controller.
   *
   * @param controller Output controller.
   */
  explicit OutputLoop(OutputController<Inputs, Outputs>&& controller);

  virtual ~OutputLoop() = default;

  OutputLoop(OutputLoop&&) = default;
  OutputLoop& operator=(OutputLoop&&) = default;

  /**
   * Enables the controller.
   */
  void Enable();

  /**
   * Disables the controller and zeros the control input.
   */
  void Disable();

  /**
   * Returns the controller's next reference r.
   */
  const Eigen::Matrix<double, Outputs, 1>& NextR() const;

  /**
   * Returns an element of the controller's next reference r.
   *
   * @param i Row of r.
   */
  double NextR(int i) const;

  /**
   * Returns the controller's calculated control input u.
   */
  const Eigen::Matrix<double, Inputs, 1>& U() const;

  /**
   * Returns an element of the controller's calculated control input u.
   *
   * @param i Row of u.
   */
  double U(int i) const;

  /**
   * Set the next reference r.
   *
   * @param nextR Next reference.
   */
  void SetNextR(const Eigen::Matrix<double, Outputs, 1>& nextR);

  /**
   * Return the controller used internally.
   */
  const OutputController<Inputs, Outputs>& GetController() const;

  /**
   * Zeroes reference R, controller output U, plant output Y, and state estimate
   * Xhat.
   */
  void Reset();

  /**
   * Returns difference between reference r and y.
   */
  const Eigen::Matrix<double, Outputs, 1> Error() const;

  /**
   * Sets new controller output.
   *
   * After calling this, the user should send the elements of u to the
   * actuators.
   *
   * @param y Measurement vector.
   */
  void Update(const Eigen::Matrix<double, Outputs, 1>& y);

  /**
   * Sets the current controller to be "index". This can be used for gain
   * scheduling.
   *
   * @param index The controller index.
   */
  void SetIndex(int index);

  /**
   * Returns the current controller index.
   */
  int GetIndex() const;

 protected:
  OutputController<Inputs, Outputs> m_controller;

  // Reference to go to in the next cycle (used by feedforward controller).
  Eigen::Matrix<double, Outputs, 1> m_nextR;

  // The current sensor measurements.
  Eigen::Matrix<double, Outputs, 1> m_y;

  // These are accessible from non-templated subclasses.
  static constexpr int kInputs = Inputs;
  static constexpr int kOutputs = Outputs;
};

}  // namespace frc

#include "frc/controller/OutputLoop.inc"
