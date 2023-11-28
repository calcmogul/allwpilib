// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <unsupported/Eigen/MatrixFunctions>

#include "frc/EigenCore.h"
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
  if constexpr (States != Eigen::Dynamic && Inputs != Eigen::Dynamic) {
    // M = [A  B]
    //     [0  0]
    Matrixd<States + Inputs, States + Inputs> M;
    M.template block<States, States>(0, 0) = contA;
    M.template block<States, Inputs>(0, States) = contB;
    M.template block<Inputs, States + Inputs>(States, 0).setZero();

    // ϕ = eᴹᵀ = [A_d  B_d]
    //           [ 0    I ]
    Matrixd<States + Inputs, States + Inputs> phi = (M * dt.value()).exp();

    *discA = phi.template block<States, States>(0, 0);
    *discB = phi.template block<States, Inputs>(0, States);
  } else {
    int states = contA.rows();
    int inputs = contB.cols();

    // M = [A  B]
    //     [0  0]
    Eigen::MatrixXd M{states + inputs, states + inputs};
    M.block(0, 0, states, states) = contA;
    M.block(0, states, states, inputs) = contB;
    M.block(states, 0, inputs, states + inputs).setZero();

    // ϕ = eᴹᵀ = [A_d  B_d]
    //           [ 0    I ]
    Eigen::MatrixXd phi = (M * dt.value()).exp();

    *discA = phi.block(0, 0, states, states);
    *discB = phi.block(0, states, states, inputs);
  }
}

/**
 * Discretizes the given continuous A and Q matrices.
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
  // Make continuous Q symmetric if it isn't already
  Matrixd<States, States> Q = (contQ + contQ.transpose()) / 2.0;

  if constexpr (States != Eigen::Dynamic) {
    // M = [−A  Q ]
    //     [ 0  Aᵀ]
    Matrixd<2 * States, 2 * States> M;
    M.template block<States, States>(0, 0) = -contA;
    M.template block<States, States>(0, States) = Q;
    M.template block<States, States>(States, 0).setZero();
    M.template block<States, States>(States, States) = contA.transpose();

    // ϕ = eᴹᵀ = [−A_d  A_d⁻¹Q_d]
    //           [ 0      A_dᵀ  ]
    Matrixd<2 * States, 2 * States> phi = (M * dt.value()).exp();

    // ϕ₁₂ = A_d⁻¹Q_d
    Matrixd<States, States> phi12 =
        phi.template block<States, States>(0, States);

    // ϕ₂₂ = A_dᵀ
    Matrixd<States, States> phi22 =
        phi.template block<States, States>(States, States);

    *discA = phi22.transpose();

    Q = *discA * phi12;
  } else {
    int states = contA.rows();

    // M = [−A  Q ]
    //     [ 0  Aᵀ]
    Eigen::MatrixXd M{2 * states, 2 * states};
    M.block(0, 0, states, states) = -contA;
    M.block(0, states, states, states) = Q;
    M.block(states, 0, states, states).setZero();
    M.block(states, states, states, states) = contA.transpose();

    // ϕ = eᴹᵀ = [−A_d  A_d⁻¹Q_d]
    //           [ 0      A_dᵀ  ]
    Eigen::MatrixXd phi = (M * dt.value()).exp();

    // ϕ₁₂ = A_d⁻¹Q_d
    Eigen::MatrixXd phi12 = phi.block(0, states, states, states);

    // ϕ₂₂ = A_dᵀ
    Eigen::MatrixXd phi22 = phi.block(states, states, states, states);

    *discA = phi22.transpose();

    Q = *discA * phi12;
  }

  // Make discrete Q symmetric if it isn't already
  *discQ = (Q + Q.transpose()) / 2.0;
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
