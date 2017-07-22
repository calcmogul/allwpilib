/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ElevatorCoeffs.h"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeElevatorPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.0044598512120532915;
  A(1, 0) = 0.0;
  A(1, 1) = 0.7757573104865231;
  Eigen::Matrix<double, 2, 2> Ainv;
  Ainv(0, 0) = 1.0;
  Ainv(0, 1) = -0.005749028918923441;
  Ainv(1, 0) = 0.0;
  Ainv(1, 1) = 1.2890629407963183;
  Eigen::Matrix<double, 2, 1> B;
  B(0, 0) = 0.0002518490070012884;
  B(1, 0) = 0.09569671214230685;
  Eigen::Matrix<double, 1, 2> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  return frc::StateSpacePlantCoeffs<2, 1, 1>(A, Ainv, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 1, 1> MakeElevatorControllerCoeffs() {
  Eigen::Matrix<double, 1, 2> K;
  K(0, 0) = 182.67594274601808;
  K(0, 1) = 7.796742212051938;
  Eigen::Matrix<double, 1, 2> Kff;
  Kff(0, 0) = 9.785918620598194;
  Kff(0, 1) = 9.296048537513542;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 1, 1> MakeElevatorObserverCoeffs() {
  Eigen::Matrix<double, 2, 1> L;
  L(0, 0) = 1.0147749573716542;
  L(1, 0) = 2.570674342540559;
  return frc::StateSpaceObserverCoeffs<2, 1, 1>(L);
}

frc::StateSpaceLoop<2, 1, 1> MakeElevatorLoop() {
  return frc::StateSpaceLoop<2, 1, 1>(MakeElevatorPlantCoeffs(),
                                      MakeElevatorControllerCoeffs(),
                                      MakeElevatorObserverCoeffs());
}
