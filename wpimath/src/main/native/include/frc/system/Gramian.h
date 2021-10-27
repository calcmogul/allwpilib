// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "drake/math/continuous_lyapunov_equation.h"
#include "wpimath/MathShared.h"

namespace frc {

/**
 * Computes the controllability Gramian for the given (A, B) pair.
 *
 * The controllability Gramian is defined as:
 *
 * W_c = ∫_0^∞ e^(Aτ) BBᵀ e^(Aᵀτ) dτ
 *
 * The controllability Gramian is the matrix W_c that satisfies the continuous
 * Lyapunov equation AW_c + W_cAᵀ + BBᵀ = 0.
 *
 * See https://en.wikipedia.org/wiki/Controllability_Gramian for more.
 *
 * @param A System matrix.
 * @param B Input matrix.
 * @throws std::invalid_argument if A is unstable.
 */
template <int States, int Inputs>
Eigen::Matrix<double, States, States> ControllabilityGramian(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B) {
  // If A is unstable, the Gramian cannot be computed
  Eigen::EigenSolver<Eigen::Matrix<double, States, States>> es{A};
  for (int i = 0; i < States; ++i) {
    if (es.eigenvalues()[i].real() * es.eigenvalues()[i].real() +
            es.eigenvalues()[i].imag() * es.eigenvalues()[i].imag() >=
        1) {
      wpi::math::MathSharedStore::ReportError(
          "The system matrix passed to ControllabilityGramian() is unstable!");
      throw std::invalid_argument(
          "The system matrix passed to ControllabilityGramian() is unstable!");
    }
  }

  return drake::math::RealContinuousLyapunovEquation(A, B * B.transpose());
}

/**
 * Computes the observability Gramian for the given (A, C) pair.
 *
 * The observability Gramian is defined as:
 *
 * W_o = ∫_0^∞ e^(Aᵀτ) CᵀC e^(Aτ) dτ
 *
 * The observability Gramian is the matrix W_o that satisfies the continuous
 * Lyapunov equation AᵀW_o + W_oA + CᵀC = 0.
 *
 * See https://en.wikipedia.org/wiki/Observability_Gramian for more.
 *
 * @param A System matrix.
 * @param C Output matrix.
 * @throws std::invalid_argument if A is unstable.
 */
template <int States, int Outputs>
Eigen::Matrix<double, States, States> ObservabilityGramian(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, Outputs, States>& C) {
  // If A is unstable, the Gramian cannot be computed
  Eigen::EigenSolver<Eigen::Matrix<double, States, States>> es{A};
  for (int i = 0; i < States; ++i) {
    if (es.eigenvalues()[i].real() * es.eigenvalues()[i].real() +
            es.eigenvalues()[i].imag() * es.eigenvalues()[i].imag() >=
        1) {
      wpi::math::MathSharedStore::ReportError(
          "The system matrix passed to ObservabilityGramian() is unstable!");
      throw std::invalid_argument(
          "The system matrix passed to ObservabilityGramian() is unstable!");
    }
  }

  return drake::math::RealContinuousLyapunovEquation(A.transpose(),
                                                     C.transpose() * C);
}

}  // namespace frc
