/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ElevatorCoeffs.h"

#include <Eigen/Core>

frc::PeriodVariantPlantCoeffs<2, 1, 1> MakeElevatorPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> Acontinuous;
  Acontinuous(0, 0) = 0.0;
  Acontinuous(0, 1) = 1.0;
  Acontinuous(1, 0) = 0.0;
  Acontinuous(1, 1) = -50.280307313265006;
  Eigen::Matrix<double, 2, 1> Bcontinuous;
  Bcontinuous(0, 0) = 0.0;
  Bcontinuous(1, 0) = 21.457377744727182;
  Eigen::Matrix<double, 1, 2> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  return frc::PeriodVariantPlantCoeffs<2, 1, 1>(Acontinuous, Bcontinuous, C, D);
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

frc::PeriodVariantKalmanFilterCoeffs<2, 1, 1> MakeElevatorObserverCoeffs() {
  Eigen::Matrix<double, 2, 2> Qcontinuous;
  Qcontinuous(0, 0) = 0.0025000000000000005;
  Qcontinuous(0, 1) = 0.0;
  Qcontinuous(1, 0) = 0.0;
  Qcontinuous(1, 1) = 1.0;
  Eigen::Matrix<double, 1, 1> Rcontinuous;
  Rcontinuous(0, 0) = 1e-08;
  Eigen::Matrix<double, 2, 2> PsteadyState;
  PsteadyState(0, 0) = 9.999960762186367e-09;
  PsteadyState(0, 1) = 3.3137610277699093e-08;
  PsteadyState(1, 0) = 3.313761027731732e-08;
  PsteadyState(1, 1) = 2.4410161689007195;
  return frc::PeriodVariantKalmanFilterCoeffs<2, 1, 1>(Qcontinuous, Rcontinuous,
                                                       PsteadyState);
}

frc::PeriodVariantLoop<2, 1, 1> MakeElevatorLoop() {
  return frc::PeriodVariantLoop<2, 1, 1>(MakeElevatorPlantCoeffs(),
                                         MakeElevatorControllerCoeffs(),
                                         MakeElevatorObserverCoeffs());
}
