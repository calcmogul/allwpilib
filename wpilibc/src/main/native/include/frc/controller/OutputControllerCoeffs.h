/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

namespace frc {

/**
 * A container for all the output controller coefficients.
 */
template <int Inputs, int Outputs>
struct OutputControllerCoeffs final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Controller gain matrix.
   */
  const Eigen::Matrix<double, Inputs, Outputs> K;

  /**
   * Minimum control input.
   */
  const Eigen::Matrix<double, Inputs, 1> Umin;

  /**
   * Maximum control input.
   */
  const Eigen::Matrix<double, Inputs, 1> Umax;

  /**
   * Construct the container for the controller coefficients.
   *
   * @param K    Controller gain matrix.
   * @param Umin Minimum control input.
   * @param Umax Maximum control input.
   */
  OutputControllerCoeffs(const Eigen::Matrix<double, Inputs, Outputs>& K,
                         const Eigen::Matrix<double, Inputs, 1>& Umin,
                         const Eigen::Matrix<double, Inputs, 1>& Umax);
};

}  // namespace frc

#include "frc/controller/OutputControllerCoeffs.inc"
