// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

#include "frc/EigenCore.h"
#include "frc/fmt/Eigen.h"
#include "units/time.h"

namespace frc {

/**
 * Discretizes the given continuous A matrix.
 *
 * @tparam States Number of states.
 * @param contA Continuous system matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 */
template <int States>
void DiscretizeA(const Matrixd<States, States>& contA, units::second_t dt,
                 Matrixd<States, States>* discA) {
  // A_d = eᴬᵀ
  *discA = (contA * dt.value()).exp();
}

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template <int States, int Inputs>
void DiscretizeAB(const Matrixd<States, States>& contA,
                  const Matrixd<States, Inputs>& contB, units::second_t dt,
                  Matrixd<States, States>* discA,
                  Matrixd<States, Inputs>* discB) {
  // A_d = eᴬᵀ
  // B_d = A⁻¹(eᴬᵀ − I)B
  //     = (A⁻¹eᴬᵀ − A⁻¹)B
  //     = A⁻¹eᴬᵀB − A⁻¹B
  *discA = (contA * dt.value()).exp();
  *discB = contA.partialPivLu().solve(*discA -
                                      Matrixd<States, States>::Identity()) *
           contB;
}

/**
 * Discretizes the given continuous A and Q matrices.
 *
 * Rather than solving a 2N x 2N matrix exponential like in DiscretizeAQ()
 * (which is expensive), we take advantage of the structure of the block matrix
 * of A and Q.
 *
 * <ul>
 *   <li>eᴬᵀ, which is only N x N, is relatively cheap.
 *   <li>The upper-right quarter of the 2N x 2N matrix, which we can approximate
 *       using a taylor series to several terms and still be substantially
 *       cheaper than taking the big exponential.
 * </ul>
 *
 * @tparam States Number of states.
 * @param contA Continuous system matrix.
 * @param contQ Continuous process noise covariance matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discQ Storage for discrete process noise covariance matrix.
 */
template <int States>
void DiscretizeAQ(const Matrixd<States, States>& contA,
                  const Matrixd<States, States>& contQ, units::second_t dt,
                  Matrixd<States, States>* discA,
                  Matrixd<States, States>* discQ) {
  //       T
  // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //       0
  //
  // M = [−A  Q ]
  //     [ 0  Aᵀ]
  // ϕ = eᴹᵀ = [−A_d  A_d⁻¹Q_d]
  //           [ 0      A_dᵀ  ]
  // ϕ₁₂ = A_d⁻¹Q_d
  //
  // Taylor series of ϕ:
  //
  //   ϕ = eᴹᵀ = I + MT + 1/2 M²T² + 1/6 M³T³ + …
  //   ϕ = eᴹᵀ = I + MT + 1/2 T²M² + 1/6 T³M³ + …
  //
  // Taylor series of ϕ expanded for ϕ₁₂:
  //
  //   ϕ₁₁ = I + TM₁₁ + 1/2 T²M²₁₁ + 1/6 T³M³₁₁ + …
  //   ϕ₁₂ = 0 + TM₁₂ + 1/2 T²M²₁₂ + 1/6 T³M³₁₂ + …
  //
  // M = [−A  Q ]
  //     [ 0  Aᵀ]
  //
  // Let B = −A, C = Q, D = Aᵀ.
  //
  //   [E  F][B  C] = [EB  EC + FD]
  //   [0  G][0  D]   [0     GD   ]
  //                = [−EA  EQ + FAᵀ]
  //                  [0       GD   ]

  Matrixd<States, States> F = contQ;
  Matrixd<States, States> E = -contA;

  double coeff = dt.value();
  Matrixd<States, States> phi11 = Matrixd<States, States>::Identity();
  Matrixd<States, States> phi12 = Matrixd<States, States>::Zero();

  for (int i = 1; (F * coeff).norm() > 1e-10 * phi12.norm(); ++i) {
    phi11 += E * coeff;
    phi12 += F * coeff;

    // Fₖ₊₁ = EₖQ + FₖAᵀ
    F = E * contQ + F * contA.transpose();

    // Eₖ₊₁ = Eₖ⋅−A
    E *= -contA;
    coeff *= dt.value() / static_cast<double>(i + 1);
  }

  *discA = -phi11;
  fmt::print("discA actual:\n{}\n", *discA);
  DiscretizeA<States>(contA, dt, discA);
  fmt::print("discA expected:\n{}\n", *discA);
  *discQ = *discA * phi12;
}

/**
 * Returns a discretized version of the provided continuous measurement noise
 * covariance matrix.
 *
 * @tparam Outputs Number of outputs.
 * @param R  Continuous measurement noise covariance matrix.
 * @param dt Discretization timestep.
 */
template <int Outputs>
Matrixd<Outputs, Outputs> DiscretizeR(const Matrixd<Outputs, Outputs>& R,
                                      units::second_t dt) {
  // R_d = 1/T R
  return R / dt.value();
}

}  // namespace frc
