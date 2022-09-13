// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/array.h>

#include "frc/EigenCore.h"
#include "frc/optimization/Problem.h"
#include "units/current.h"

namespace frc {

/**
 * This class computes the optimal current allocation for a list of subsystems
 * given a list of their desired currents and current tolerances that determine
 * which subsystem gets less current if the current budget is exceeded.
 * Subsystems with a smaller tolerance are given higher priority.
 *
 * @tparam Subsystems The number of subsystems.
 */
template <int Subsystems>
class CurrentManager {
 public:
  /**
   * Constructs a CurrentManager.
   *
   * @param currentTolerances The relative current tolerance of each subsystem.
   * @param maxCurrent The current budget to allocate between subsystems.
   */
  CurrentManager(
      const wpi::array<units::ampere_t, Subsystems>& currentTolerances,
      units::ampere_t maxCurrent)
      : m_desiredCurrents{Subsystems, 1},
        m_allocatedCurrents{m_problem.DecisionVariable(Subsystems, 1)} {
    VariableMatrix J = 0.0;
    for (int i = 0; i < Subsystems; ++i) {
      // The weight is 1/tolᵢ² where tolᵢ is the tolerance between the desired
      // and allocated current for subsystem i
      auto error = m_desiredCurrents(i) - m_allocatedCurrents(i);
      J += error * error /
           (currentTolerances[i].value() * currentTolerances[i].value());
    }
    m_problem.Minimize(J);

    // Currents must be nonnegative
    for (int i = 0; i < Subsystems; ++i) {
      m_problem.SubjectTo(m_allocatedCurrents(i) >= 0);
    }

    // Keep total current below maximum
    VariableMatrix currentSum = 0.0;
    for (int i = 0; i < Subsystems; ++i) {
      currentSum += m_allocatedCurrents(i);
    }
    m_problem.SubjectTo(currentSum <= maxCurrent.value());
  }

  /**
   * Returns the optimal current allocation for a list of subsystems given a
   * list of their desired currents and current tolerances that determine which
   * subsystem gets less current if the current budget is exceeded. Subsystems
   * with a smaller tolerance are given higher priority.
   *
   * @param desiredCurrents The desired current for each subsystem.
   */
  wpi::array<units::ampere_t, Subsystems> Calculate(
      const wpi::array<units::ampere_t, Subsystems>& desiredCurrents) {
    for (int i = 0; i < Subsystems; ++i) {
      m_desiredCurrents(i) = desiredCurrents[i].value();
    }

    m_problem.Solve();

    wpi::array<units::ampere_t, Subsystems> result{wpi::empty_array};
    for (int i = 0; i < Subsystems; ++i) {
      result[i] = units::ampere_t{m_allocatedCurrents.Value(i)};
    }

    return result;
  }

 private:
  Problem m_problem;
  VariableMatrix m_desiredCurrents;
  VariableMatrix m_allocatedCurrents;
};

}  // namespace frc
