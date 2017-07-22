/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cmath>

#include "subsystems/Drivetrain.h"

DrivetrainController::DrivetrainController() { m_Y.setZero(); }

void DrivetrainController::Enable() { m_loop.Enable(); }

void DrivetrainController::Disable() { m_loop.Disable(); }

void DrivetrainController::SetReferences(double leftPosition,
                                         double leftVelocity,
                                         double rightPosition,
                                         double rightVelocity) {
  Eigen::Matrix<double, 4, 1> nextR;
  nextR << leftPosition, leftVelocity, rightPosition, rightVelocity;
  m_loop.SetNextR(nextR);
}

bool DrivetrainController::AtReferences() const { return m_atReferences; }

void DrivetrainController::SetMeasuredStates(double leftPosition,
                                             double rightPosition) {
  m_Y << leftPosition, rightPosition;
}

double DrivetrainController::ControllerLeftVoltage() const {
  return m_loop.U(0);
}

double DrivetrainController::ControllerRightVoltage() const {
  return m_loop.U(1);
}

double DrivetrainController::EstimatedLeftPosition() const {
  return m_loop.Xhat(0);
}

double DrivetrainController::EstimatedLeftVelocity() const {
  return m_loop.Xhat(1);
}

double DrivetrainController::EstimatedRightPosition() const {
  return m_loop.Xhat(2);
}

double DrivetrainController::EstimatedRightVelocity() const {
  return m_loop.Xhat(3);
}

double DrivetrainController::LeftPositionError() const {
  return m_loop.Error()(0, 0);
}

double DrivetrainController::LeftVelocityError() const {
  return m_loop.Error()(1, 0);
}

double DrivetrainController::RightPositionError() const {
  return m_loop.Error()(2, 0);
}

double DrivetrainController::RightVelocityError() const {
  return m_loop.Error()(3, 0);
}

void DrivetrainController::Update(std::chrono::nanoseconds dt) {
  m_loop.Correct(m_Y);

  auto error = m_loop.Error();
  m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                   std::abs(error(1, 0)) < kVelocityTolerance &&
                   std::abs(error(2, 0)) < kPositionTolerance &&
                   std::abs(error(3, 0)) < kVelocityTolerance;

  m_loop.Predict(dt);
}

void DrivetrainController::Reset() { m_loop.Reset(); }
