/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SingleJointedArmCoeffs.h"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeSingleJointedArmPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.005049999879023277;
  A(1, 0) = 0.0;
  A(1, 1) = 0.9999999520884271;
  Eigen::Matrix<double, 2, 2> Ainv;
  Ainv(0, 0) = 1.0;
  Ainv(0, 1) = -0.005050000120976726;
  Ainv(1, 0) = 0.0;
  Ainv(1, 1) = 1.0000000479115752;
  Eigen::Matrix<double, 2, 1> B;
  B(0, 0) = 1.1444081154628273e-07;
  B(1, 0) = 4.5323093319779346e-05;
  Eigen::Matrix<double, 1, 2> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  return frc::StateSpacePlantCoeffs<2, 1, 1>(A, Ainv, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 1, 1>
MakeSingleJointedArmControllerCoeffs() {
  Eigen::Matrix<double, 1, 2> K;
  K(0, 0) = 681.2433935155831;
  K(0, 1) = 412.7588733352761;
  Eigen::Matrix<double, 1, 2> Kff;
  Kff(0, 0) = 0.05411724562879173;
  Kff(0, 1) = 0.8571064115615331;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 1, 1> MakeSingleJointedArmObserverCoeffs() {
  Eigen::Matrix<double, 2, 1> L;
  L(0, 0) = 0.8424737626970222;
  L(1, 0) = 7.738857743087447;
  return frc::StateSpaceObserverCoeffs<2, 1, 1>(L);
}

frc::StateSpaceLoop<2, 1, 1> MakeSingleJointedArmLoop() {
  return frc::StateSpaceLoop<2, 1, 1>(MakeSingleJointedArmPlantCoeffs(),
                                      MakeSingleJointedArmControllerCoeffs(),
                                      MakeSingleJointedArmObserverCoeffs());
}
