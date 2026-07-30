// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <unsupported/Eigen/MatrixFunctions>

inline void BM_MatrixPow(benchmark::State& state) {
  // NOLINTNEXTLINE(clang-analyzer-deadcode.DeadStores)
  for (auto _ : state) {
    Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Zero();
    for (int col = 1; col < 7; ++col) {
      A(col - 1, col) = 1;
    }
    Eigen::Matrix<double, 7, 7> result = A.pow(0.5);
    benchmark::DoNotOptimize(result);
  }
}
