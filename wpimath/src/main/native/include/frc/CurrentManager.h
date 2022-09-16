// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/span.h>

#include "frc/optimization/Problem.h"
#include "units/current.h"

namespace frc {

/**
 * This class computes the optimal current allocation for a list of subsystems
 * given a list of their desired currents and current tolerances that determine
 * which subsystem gets less current if the current budget is exceeded.
 * Subsystems with a smaller tolerance are given higher priority.
 */
class CurrentManager {
 public:
  /**
   * Constructs a CurrentManager.
   *
   * @param currentTolerances The relative current tolerance of each subsystem.
   * @param maxCurrent The current budget to allocate between subsystems.
   */
  CurrentManager(wpi::span<const units::ampere_t> currentTolerances,
                 units::ampere_t maxCurrent);

  /**
   * Returns the optimal current allocation for a list of subsystems given a
   * list of their desired currents and current tolerances that determine which
   * subsystem gets less current if the current budget is exceeded. Subsystems
   * with a smaller tolerance are given higher priority.
   *
   * @param desiredCurrents The desired current for each subsystem.
   * @throws std::runtime_error if the number of desired currents doesn't equal
   *         the number of tolerances passed in the constructor.
   */
  std::vector<units::ampere_t> Calculate(
      wpi::span<const units::ampere_t> desiredCurrents);

 private:
  Problem m_problem;
  VariableMatrix m_desiredCurrents;
  VariableMatrix m_allocatedCurrents;
};

}  // namespace frc
