// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Cholesky>
#include <fmt/format.h>

#include "frc/DARE.h"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/KalmanFilter.h"
#include "frc/fmt/Eigen.h"
#include "frc/system/Discretization.h"
#include "wpimath/MathShared.h"

namespace frc {

template <int States, int Inputs, int Outputs>
KalmanFilter<States, Inputs, Outputs>::KalmanFilter(
    LinearSystem<States, Inputs, Outputs>& plant,
    const StateArray& stateStdDevs, const OutputArray& measurementStdDevs,
    units::second_t dt) {
  m_plant = &plant;

  m_contQ = MakeCovMatrix(stateStdDevs);
  m_contR = MakeCovMatrix(measurementStdDevs);
  m_dt = dt;

  // Find discrete A and Q
  Matrixd<States, States> discA;
  Matrixd<States, States> discQ;
  DiscretizeAQ<States>(plant.A(), m_contQ, dt, &discA, &discQ);

  Matrixd<Outputs, Outputs> discR = DiscretizeR<Outputs>(m_contR, dt);

  const auto& C = plant.C();

  if (!IsDetectable<States, Outputs>(discA, C)) {
    auto unobservableStates = GetUnobservableStates<States, Outputs>(discA, C);

    std::string msg = fmt::format(
        "The system passed to the Kalman filter is "
        "unobservable!\n\nA =\n{}\nC =\n{}\n",
        discA, C);
    if (!unobservableStates.empty()) {
      msg += fmt::format("\n0-based indices of unobservable states: {}\n",
                         fmt::join(unobservableStates, ", "));
    }

    wpi::math::MathSharedStore::ReportError(msg);
    throw std::invalid_argument(msg);
  }

  m_initP =
      DARE<States, Outputs>(discA.transpose(), C.transpose(), discQ, discR);

  Reset();
}

template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::Predict(const InputVector& u,
                                                    units::second_t dt) {
  // Find discrete A and Q
  StateMatrix discA;
  StateMatrix discQ;
  DiscretizeAQ<States>(m_plant->A(), m_contQ, dt, &discA, &discQ);

  m_xHat = m_plant->CalculateX(m_xHat, u, dt);

  // Pₖ₊₁⁻ = APₖ⁻Aᵀ + Q
  m_P = discA * m_P * discA.transpose() + discQ;

  m_dt = dt;
}

template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::Correct(
    const InputVector& u, const OutputVector& y,
    const Matrixd<Outputs, Outputs>& R) {
  const auto& C = m_plant->C();
  const auto& D = m_plant->D();

  const Matrixd<Outputs, Outputs> discR = DiscretizeR<Outputs>(R, m_dt);

  Matrixd<Outputs, Outputs> S = C * m_P * C.transpose() + discR;

  // We want to put K = PCᵀS⁻¹ into Ax = b form so we can solve it more
  // efficiently.
  //
  // K = PCᵀS⁻¹
  // KS = PCᵀ
  // (KS)ᵀ = (PCᵀ)ᵀ
  // SᵀKᵀ = CPᵀ
  //
  // The solution of Ax = b can be found via x = A.solve(b).
  //
  // Kᵀ = Sᵀ.solve(CPᵀ)
  // K = (Sᵀ.solve(CPᵀ))ᵀ
  Matrixd<States, Outputs> K =
      S.transpose().ldlt().solve(C * m_P.transpose()).transpose();

  // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − (Cx̂ₖ₊₁⁻ + Duₖ₊₁))
  m_xHat += K * (y - (C * m_xHat + D * u));

  // Pₖ₊₁⁺ = (I−Kₖ₊₁C)Pₖ₊₁⁻(I−Kₖ₊₁C)ᵀ + Kₖ₊₁RKₖ₊₁ᵀ
  // Use Joseph form for numerical stability
  m_P = (StateMatrix::Identity() - K * C) * m_P *
            (StateMatrix::Identity() - K * C).transpose() +
        K * discR * K.transpose();
}

}  // namespace frc
