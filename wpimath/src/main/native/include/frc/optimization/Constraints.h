// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/Variable.h"
#include "frc/optimization/VariableMatrix.h"

namespace frc {

/**
 * An equality constraint has the form c(x) = 0.
 */
struct WPILIB_DLLEXPORT EqualityConstraints {
  std::vector<autodiff::Variable> constraints;
};

/**
 * An inequality constraint has the form c(x) ≥ 0.
 */
struct WPILIB_DLLEXPORT InequalityConstraints {
  std::vector<autodiff::Variable> constraints;
};

WPILIB_DLLEXPORT EqualityConstraints operator==(const VariableMatrix& lhs,
                                                const VariableMatrix& rhs);

WPILIB_DLLEXPORT InequalityConstraints operator<(const VariableMatrix& lhs,
                                                 const VariableMatrix& rhs);

WPILIB_DLLEXPORT InequalityConstraints operator<=(const VariableMatrix& lhs,
                                                  const VariableMatrix& rhs);

WPILIB_DLLEXPORT InequalityConstraints operator>(const VariableMatrix& lhs,
                                                 const VariableMatrix& rhs);

WPILIB_DLLEXPORT InequalityConstraints operator>=(const VariableMatrix& lhs,
                                                  const VariableMatrix& rhs);

}  // namespace frc
