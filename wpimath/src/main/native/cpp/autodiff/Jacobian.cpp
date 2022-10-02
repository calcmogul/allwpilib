// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Jacobian.h"

#include <tuple>
#include <vector>

#include "frc/autodiff/Gradient.h"

namespace frc::autodiff {

Eigen::SparseMatrix<double> Jacobian(Eigen::Ref<VectorXvar> variables,
                                     Eigen::Ref<VectorXvar> wrt) {
  Eigen::SparseMatrix<double> J{variables.rows(), wrt.rows()};

  // Reserve triplet space for 1% sparsity
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(variables.rows() * wrt.rows() / 100);

  for (int row = 0; row < variables.rows(); ++row) {
    Eigen::SparseVector<double> g = Gradient(variables(row), wrt);
    for (decltype(g)::InnerIterator it{g}; it; ++it) {
      triplets.emplace_back(row, it.index(), it.value());
    }
  }
  J.setFromTriplets(triplets.begin(), triplets.end());

  return J;
}

}  // namespace frc::autodiff
