// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <wpi/print.h>

#include "frc/fmt/Eigen.h"

inline void ExpectMatrixEqual(const Eigen::MatrixXd& lhs,
                              const Eigen::MatrixXd& rhs, double tolerance) {
  for (int row = 0; row < lhs.rows(); ++row) {
    for (int col = 0; col < lhs.cols(); ++col) {
      EXPECT_NEAR(lhs(row, col), rhs(row, col), tolerance)
          << fmt::format("row = {}, col = {}", row, col);
    }
  }

  if (::testing::Test::HasFailure()) {
    wpi::print("lhs =\n{}\n", lhs);
    wpi::print("rhs =\n{}\n", rhs);
    wpi::print("delta =\n{}\n", Eigen::MatrixXd{lhs - rhs});
  }
}

inline void ExpectPositiveSemidefinite(
    const Eigen::Ref<const Eigen::MatrixXd>& X) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigX{X,
                                                      Eigen::EigenvaluesOnly};
  for (int i = 0; i < X.rows(); ++i) {
    EXPECT_GE(eigX.eigenvalues()[i], 0.0);
  }
}
