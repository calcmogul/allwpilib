// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "Eigen/SparseCore"
#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

/**
 * Returns the Jacobian of an autodiff vector with respect to the given vector.
 *
 * @param variables Vector of which to compute the Jacobian.
 * @param wrt Vector with respect to which to compute the Jacobian.
 */
WPILIB_DLLEXPORT Eigen::SparseMatrix<double> Jacobian(VectorXvar& variables,
                                                      VectorXvar& wrt);

}  // namespace frc::autodiff
