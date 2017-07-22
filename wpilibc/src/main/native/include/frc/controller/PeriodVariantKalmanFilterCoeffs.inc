/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>::
    PeriodVariantKalmanFilterCoeffs(
        const Eigen::Matrix<double, States, States>& Qcontinuous,
        const Eigen::Matrix<double, Outputs, Outputs>& Rcontinuous,
        const Eigen::Matrix<double, States, States>& PsteadyState)
    : Qcontinuous(Qcontinuous),
      Rcontinuous(Rcontinuous),
      PsteadyState(PsteadyState) {}

}  // namespace frc
