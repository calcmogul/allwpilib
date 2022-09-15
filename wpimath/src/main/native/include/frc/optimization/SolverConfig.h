// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/time.h"

namespace frc {

/**
 * Solver configuration.
 */
struct SolverConfig {
  /// The solver will stop once the error is below this tolerance.
  double tolerance = 1e-6;

  /// The maximum number of solver iterations before returning a solution.
  int maxIterations = 1000;

  /// The maximum elapsed wall clock time before returning a solution.
  units::second_t timeout = 10_s;
};

}  // namespace frc
