/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DifferentialDriveCoeffs.h"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDifferentialDrivePlantCoeffs() {
  Eigen::Matrix<double, 4, 4> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.0050499899190335415;
  A(0, 2) = 0.0;
  A(0, 3) = -7.242747529641817e-08;
  A(1, 0) = 0.0;
  A(1, 1) = 0.9999960076778236;
  A(1, 2) = 0.0;
  A(1, 3) = -2.868411045809369e-05;
  A(2, 0) = 0.0;
  A(2, 1) = -7.242747529641817e-08;
  A(2, 2) = 1.0;
  A(2, 3) = 0.0050499899190335415;
  A(3, 0) = 0.0;
  A(3, 1) = -2.8684110458093693e-05;
  A(3, 2) = 0.0;
  A(3, 3) = 0.9999960076778235;
  Eigen::Matrix<double, 4, 4> Ainv;
  Ainv(0, 0) = 1.0;
  Ainv(0, 1) = -0.005050010082378314;
  Ainv(0, 2) = 0.0;
  Ainv(0, 3) = -7.242786087636338e-08;
  Ainv(1, 0) = 0.0;
  Ainv(1, 1) = 1.0000039931609033;
  Ainv(1, 2) = 0.0;
  Ainv(1, 3) = 2.8684339515486883e-05;
  Ainv(2, 0) = 0.0;
  Ainv(2, 1) = -7.24278608763634e-08;
  Ainv(2, 2) = 1.0;
  Ainv(2, 3) = -0.005050010082378314;
  Ainv(3, 0) = 0.0;
  Ainv(3, 1) = 2.8684339515486887e-05;
  Ainv(3, 2) = 0.0;
  Ainv(3, 3) = 1.0000039931609033;
  Eigen::Matrix<double, 4, 2> B;
  B(0, 0) = 7.712552122803026e-07;
  B(0, 1) = 1.0734873601095388e-07;
  B(1, 0) = 0.0003054472023205088;
  B(1, 1) = 4.251286235223679e-05;
  B(2, 0) = 1.073487360109539e-07;
  B(2, 1) = 7.712552122803028e-07;
  B(3, 0) = 4.2512862352236795e-05;
  B(3, 1) = 0.00030544720232050885;
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
  return frc::StateSpacePlantCoeffs<4, 2, 2>(A, Ainv, B, C, D);
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

frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDifferentialDriveObserverCoeffs() {
  Eigen::Matrix<double, 4, 2> L;
  L(0, 0) = 1.096020218941191;
  L(0, 1) = -2.8683085835869126e-05;
  L(1, 0) = 19.014582877804802;
  L(1, 1) = -0.005952496568204303;
  L(2, 0) = -2.8683085835949232e-05;
  L(2, 1) = 1.0960202189411916;
  L(3, 0) = -0.005952496568220164;
  L(3, 1) = 19.01458287780488;
  return frc::StateSpaceObserverCoeffs<4, 2, 2>(L);
}

frc::StateSpaceLoop<4, 2, 2> MakeDifferentialDriveLoop() {
  return frc::StateSpaceLoop<4, 2, 2>(MakeDifferentialDrivePlantCoeffs(),
                                      MakeDifferentialDriveControllerCoeffs(),
                                      MakeDifferentialDriveObserverCoeffs());
}
