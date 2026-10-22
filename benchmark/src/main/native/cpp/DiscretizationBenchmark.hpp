// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cstdlib>

#include <benchmark/benchmark.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "wpi/math/linalg/EigenCore.hpp"
#include "wpi/math/system/Discretization.hpp"
#include "wpi/units/time.hpp"

// Reference implementation that takes the dense exponential of [A B; 0 0]T
template <int States, int Inputs>
void DiscretizeABDense(const wpi::math::Matrixd<States, States>& contA,
                       const wpi::math::Matrixd<States, Inputs>& contB,
                       wpi::units::second_t dt,
                       wpi::math::Matrixd<States, States>* discA,
                       wpi::math::Matrixd<States, Inputs>* discB) {
  wpi::math::Matrixd<States + Inputs, States + Inputs> M;
  M.template block<States, States>(0, 0) = contA;
  M.template block<States, Inputs>(0, States) = contB;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  wpi::math::Matrixd<States + Inputs, States + Inputs> phi =
      (M * dt.value()).exp();

  *discA = phi.template block<States, States>(0, 0);
  *discB = phi.template block<States, Inputs>(0, States);
}

// Random A and B scaled so ‖[A B]T‖₁ equals state.range(0) / 1000 with T = 1 s.
// The norm selects the Padé degree and number of squarings.
template <int States, int Inputs>
void MakeDiscretizationProblem(const benchmark::State& state,
                               wpi::math::Matrixd<States, States>* A,
                               wpi::math::Matrixd<States, Inputs>* B) {
  std::srand(0);
  *A = wpi::math::Matrixd<States, States>::Random();
  *B = wpi::math::Matrixd<States, Inputs>::Random();
  double norm = std::max(A->cwiseAbs().colwise().sum().maxCoeff(),
                         B->cwiseAbs().colwise().sum().maxCoeff());
  double scale = state.range(0) / 1000.0 / norm;
  *A *= scale;
  *B *= scale;
}

template <int States, int Inputs>
void BM_DiscretizeAB_Dense(benchmark::State& state) {
  wpi::math::Matrixd<States, States> A;
  wpi::math::Matrixd<States, Inputs> B;
  MakeDiscretizationProblem(state, &A, &B);

  wpi::math::Matrixd<States, States> discA;
  wpi::math::Matrixd<States, Inputs> discB;
  for (auto _ : state) {
    benchmark::DoNotOptimize(A);
    benchmark::DoNotOptimize(B);
    DiscretizeABDense<States, Inputs>(A, B, 1_s, &discA, &discB);
    benchmark::DoNotOptimize(discA);
    benchmark::DoNotOptimize(discB);
  }
}

template <int States, int Inputs>
void BM_DiscretizeAB_Structured(benchmark::State& state) {
  wpi::math::Matrixd<States, States> A;
  wpi::math::Matrixd<States, Inputs> B;
  MakeDiscretizationProblem(state, &A, &B);

  wpi::math::Matrixd<States, States> discA;
  wpi::math::Matrixd<States, Inputs> discB;
  for (auto _ : state) {
    benchmark::DoNotOptimize(A);
    benchmark::DoNotOptimize(B);
    wpi::math::DiscretizeAB<States, Inputs>(A, B, 1_s, &discA, &discB);
    benchmark::DoNotOptimize(discA);
    benchmark::DoNotOptimize(discB);
  }
}

// Reference implementation that takes the dense exponential of [−A Q; 0 Aᵀ]T
template <int States>
void DiscretizeAQDense(const wpi::math::Matrixd<States, States>& contA,
                       const wpi::math::Matrixd<States, States>& contQ,
                       wpi::units::second_t dt,
                       wpi::math::Matrixd<States, States>* discA,
                       wpi::math::Matrixd<States, States>* discQ) {
  wpi::math::Matrixd<States, States> Q = (contQ + contQ.transpose()) / 2.0;

  wpi::math::Matrixd<2 * States, 2 * States> M;
  M.template block<States, States>(0, 0) = -contA;
  M.template block<States, States>(0, States) = Q;
  M.template block<States, States>(States, 0).setZero();
  M.template block<States, States>(States, States) = contA.transpose();

  wpi::math::Matrixd<2 * States, 2 * States> phi = (M * dt.value()).exp();

  wpi::math::Matrixd<States, States> phi12 =
      phi.block(0, States, States, States);
  wpi::math::Matrixd<States, States> phi22 =
      phi.block(States, States, States, States);

  *discA = phi22.transpose();
  Q = *discA * phi12;
  *discQ = (Q + Q.transpose()) / 2.0;
}

template <int States>
void BM_DiscretizeAQ_Dense(benchmark::State& state) {
  wpi::math::Matrixd<States, States> A;
  wpi::math::Matrixd<States, States> Q;
  MakeDiscretizationProblem(state, &A, &Q);

  wpi::math::Matrixd<States, States> discA;
  wpi::math::Matrixd<States, States> discQ;
  for (auto _ : state) {
    benchmark::DoNotOptimize(A);
    benchmark::DoNotOptimize(Q);
    DiscretizeAQDense<States>(A, Q, 1_s, &discA, &discQ);
    benchmark::DoNotOptimize(discA);
    benchmark::DoNotOptimize(discQ);
  }
}

template <int States>
void BM_DiscretizeAQ_Structured(benchmark::State& state) {
  wpi::math::Matrixd<States, States> A;
  wpi::math::Matrixd<States, States> Q;
  MakeDiscretizationProblem(state, &A, &Q);

  wpi::math::Matrixd<States, States> discA;
  wpi::math::Matrixd<States, States> discQ;
  for (auto _ : state) {
    benchmark::DoNotOptimize(A);
    benchmark::DoNotOptimize(Q);
    wpi::math::DiscretizeAQ<States>(A, Q, 1_s, &discA, &discQ);
    benchmark::DoNotOptimize(discA);
    benchmark::DoNotOptimize(discQ);
  }
}
