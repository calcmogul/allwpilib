// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/autodiff/Expression.h"
#include "frc/optimization/SolverExitCondition.h"

namespace frc {

/**
 * Return value of Problem::Solve() containing the detected problem type and
 * solver's return status.
 */
struct SolverStatus {
  /// The cost function type detected by the solver.
  autodiff::ExpressionType costFunctionType = autodiff::ExpressionType::kNone;

  /// The equality constraint type detected by the solver.
  autodiff::ExpressionType equalityConstraintType =
      autodiff::ExpressionType::kNone;

  /// The inequality constraint type detected by the solver.
  autodiff::ExpressionType inequalityConstraintType =
      autodiff::ExpressionType::kNone;

  /// The solver's exit condition.
  SolverExitCondition exitCondition = SolverExitCondition::kOk;
};

}  // namespace frc
