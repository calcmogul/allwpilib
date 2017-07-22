/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DifferentialDriveCoeffs.h"

#include <Eigen/Core>

frc::PeriodVariantPlantCoeffs<4, 2, 2> MakeDifferentialDrivePlantCoeffs() {
  Eigen::Matrix<double, 4, 4> Acontinuous;
  Acontinuous(0, 0) = 0.0;
  Acontinuous(0, 1) = 1.0;
  Acontinuous(0, 2) = 0.0;
  Acontinuous(0, 3) = 0.0;
  Acontinuous(1, 0) = 0.0;
  Acontinuous(1, 1) = -0.0007906418887742482;
  Acontinuous(1, 2) = 0.0;
  Acontinuous(1, 3) = -0.005680044551015477;
  Acontinuous(2, 0) = 0.0;
  Acontinuous(2, 1) = 0.0;
  Acontinuous(2, 2) = 0.0;
  Acontinuous(2, 3) = 1.0;
  Acontinuous(3, 0) = 0.0;
  Acontinuous(3, 1) = -0.005680044551015477;
  Acontinuous(3, 2) = 0.0;
  Acontinuous(3, 3) = -0.0007906418887742482;
  Eigen::Matrix<double, 4, 2> Bcontinuous;
  Bcontinuous(0, 0) = 0.0;
  Bcontinuous(0, 1) = 0.0;
  Bcontinuous(1, 0) = 0.060484836010453395;
  Bcontinuous(1, 1) = 0.00841927286942775;
  Bcontinuous(2, 0) = 0.0;
  Bcontinuous(2, 1) = 0.0;
  Bcontinuous(3, 0) = 0.00841927286942775;
  Bcontinuous(3, 1) = 0.060484836010453395;
  Eigen::Matrix<double, 2, 4> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  C(0, 2) = 0;
  C(0, 3) = 0;
  C(1, 0) = 0;
  C(1, 1) = 0;
  C(1, 2) = 1;
  C(1, 3) = 0;
  Eigen::Matrix<double, 2, 2> D;
  D(0, 0) = 0;
  D(0, 1) = 0;
  D(1, 0) = 0;
  D(1, 1) = 0;
  return frc::PeriodVariantPlantCoeffs<4, 2, 2>(Acontinuous, Bcontinuous, C, D);
}

frc::StateSpaceControllerCoeffs<4, 2, 2>
MakeDifferentialDriveControllerCoeffs() {
  Eigen::Matrix<double, 2, 4> K;
  K(0, 0) = 85.00274539055344;
  K(0, 1) = 54.86410644080571;
  K(0, 2) = -0.05096618896484127;
  K(0, 3) = -3.746916275984323;
  K(1, 0) = -0.05096618895572961;
  K(1, 1) = -3.7469162759828314;
  K(1, 2) = 85.00274539054654;
  K(1, 3) = 54.86410644079513;
  Eigen::Matrix<double, 2, 4> Kff;
  Kff(0, 0) = 4.442350766500918;
  Kff(0, 1) = 0.04398361242098099;
  Kff(0, 2) = 0.6182972417130087;
  Kff(0, 3) = 0.0061215405221850755;
  Kff(1, 0) = 0.6182972417130086;
  Kff(1, 1) = 0.006121540522185074;
  Kff(1, 2) = 4.44235076650092;
  Kff(1, 3) = 0.043983612420981;
  Eigen::Matrix<double, 2, 1> Umin;
  Umin(0, 0) = -12.0;
  Umin(1, 0) = -12.0;
  Eigen::Matrix<double, 2, 1> Umax;
  Umax(0, 0) = 12.0;
  Umax(1, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<4, 2, 2>(K, Kff, Umin, Umax);
}

frc::PeriodVariantKalmanFilterCoeffs<4, 2, 2>
MakeDifferentialDriveObserverCoeffs() {
  Eigen::Matrix<double, 4, 4> Qcontinuous;
  Qcontinuous(0, 0) = 0.0025000000000000005;
  Qcontinuous(0, 1) = 0.0;
  Qcontinuous(0, 2) = 0.0;
  Qcontinuous(0, 3) = 0.0;
  Qcontinuous(1, 0) = 0.0;
  Qcontinuous(1, 1) = 1.0;
  Qcontinuous(1, 2) = 0.0;
  Qcontinuous(1, 3) = 0.0;
  Qcontinuous(2, 0) = 0.0;
  Qcontinuous(2, 1) = 0.0;
  Qcontinuous(2, 2) = 0.0025000000000000005;
  Qcontinuous(2, 3) = 0.0;
  Qcontinuous(3, 0) = 0.0;
  Qcontinuous(3, 1) = 0.0;
  Qcontinuous(3, 2) = 0.0;
  Qcontinuous(3, 3) = 1.0;
  Eigen::Matrix<double, 2, 2> Rcontinuous;
  Rcontinuous(0, 0) = 1e-08;
  Rcontinuous(0, 1) = 0.0;
  Rcontinuous(1, 0) = 0.0;
  Rcontinuous(1, 1) = 1e-08;
  Eigen::Matrix<double, 4, 4> PsteadyState;
  PsteadyState(0, 0) = 9.99996384154934e-09;
  PsteadyState(0, 1) = 1.901465863587794e-07;
  PsteadyState(0, 2) = -1.0371559461588366e-18;
  PsteadyState(0, 3) = -5.40709958665697e-11;
  PsteadyState(1, 0) = 1.901465863634055e-07;
  PsteadyState(1, 1) = 10.413219859335985;
  PsteadyState(1, 2) = -5.407099586703075e-11;
  PsteadyState(1, 3) = -0.002811793912471262;
  PsteadyState(2, 0) = -1.0371417215324667e-18;
  PsteadyState(2, 1) = -5.4070995864480396e-11;
  PsteadyState(2, 2) = 9.999963841242302e-09;
  PsteadyState(2, 3) = 1.9014658635294198e-07;
  PsteadyState(3, 0) = -5.407099586663918e-11;
  PsteadyState(3, 1) = -0.0028117939124712617;
  PsteadyState(3, 2) = 1.901465863582419e-07;
  PsteadyState(3, 3) = 10.413219859335985;
  return frc::PeriodVariantKalmanFilterCoeffs<4, 2, 2>(Qcontinuous, Rcontinuous,
                                                       PsteadyState);
}

frc::PeriodVariantLoop<4, 2, 2> MakeDifferentialDriveLoop() {
  return frc::PeriodVariantLoop<4, 2, 2>(
      MakeDifferentialDrivePlantCoeffs(),
      MakeDifferentialDriveControllerCoeffs(),
      MakeDifferentialDriveObserverCoeffs());
}
