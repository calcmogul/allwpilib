// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * Solver return status.
 */
enum class SolverStatus {
  /// The solver found an optimal solution.
  kOk,
  /// The solver determined the problem to be infeasible and gave up.
  kInfeasible,
  /// The solver returned its solution so far after exceeding the maximum number
  /// of iterations.
  kMaxIterations,
  /// The solver returned its solution so far after exceeding the maximum
  /// elapsed wall clock time.
  kTimeout
};

}  // namespace frc
