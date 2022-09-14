// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "Eigen/Core"
#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

/**
 * Returns gradient of a variable with respect to the given variable.
 *
 * @param variable Variable of which to compute the gradient.
 * @param wrt Variable with respect to which to compute the gradient.
 */
WPILIB_DLLEXPORT double Gradient(Variable variable, Variable& wrt);

/**
 * Returns gradient of a variable with respect to the given variable.
 *
 * @param variable Variable of which to compute the gradient.
 * @param wrt Variables with respect to which to compute the gradient.
 */
WPILIB_DLLEXPORT Eigen::VectorXd Gradient(Variable variable, VectorXvar& wrt);

}  // namespace frc::autodiff
