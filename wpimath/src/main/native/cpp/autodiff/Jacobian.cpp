// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Jacobian.h"

#include <tuple>
#include <vector>

#include "frc/autodiff/Gradient.h"

namespace frc::autodiff {

Eigen::SparseMatrix<double> Jacobian(VectorXvar& variables, VectorXvar& wrt) {
  Eigen::SparseMatrix<double> J{variables.rows(), wrt.rows()};

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < variables.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(variables(row), wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        triplets.emplace_back(row, col, g(col));
      }
    }
  }
  J.setFromTriplets(triplets.begin(), triplets.end());

  return J;
}

}  // namespace frc::autodiff
