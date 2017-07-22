/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

template <int Inputs, int Outputs>
OutputControllerCoeffs<Inputs, Outputs>::OutputControllerCoeffs(
    const Eigen::Matrix<double, Inputs, Outputs>& K,
    const Eigen::Matrix<double, Inputs, 1>& Umin,
    const Eigen::Matrix<double, Inputs, 1>& Umax)
    : K(K), Umin(Umin), Umax(Umax) {}

}  // namespace frc
