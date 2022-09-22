// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * The type of optimization problem to solve.
 */
enum class ProblemType {
  /// The optimization problem has a constant cost function and no constraints.
  kConstant,
  /// The optimization problem has a linear cost function and linear
  /// constraints.
  kLinear,
  /// The optimization problem has a quadratic cost function and linear
  /// constraints.
  kQuadratic,
  /// The optimization problem has a nonlinear cost function or nonlinear
  /// constraints.
  kNonlinear
};

}  // namespace frc
