// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Hessian.h"

#include <tuple>

#include <wpi/IntrusiveSharedPtr.h>

#include "frc/autodiff/Gradient.h"

using namespace frc::autodiff;

Hessian::Hessian(Variable variable, Eigen::Ref<VectorXvar> wrt)
    : m_variable{std::move(variable)},
      m_wrt{wrt},
      m_gradientTree{GenerateGradientTree(m_variable, m_wrt)} {
  // Reserve triplet space for 1% sparsity
  m_triplets.reserve(m_wrt.rows() * m_wrt.rows() * 0.01);
}

Eigen::SparseMatrix<double> Hessian::Calculate() {
  m_triplets.clear();
  for (int row = 0; row < m_gradientTree.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(m_gradientTree(row), m_wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        m_triplets.emplace_back(row, col, g(col));
      }
    }
  }

  Eigen::SparseMatrix<double> H{m_wrt.rows(), m_wrt.rows()};
  H.setFromTriplets(m_triplets.begin(), m_triplets.end());

  return H;
}

VectorXvar Hessian::GenerateGradientTree(Variable& variable, VectorXvar& wrt) {
  // Read wpimath/README.md#Reverse_accumulation_automatic_differentiation for
  // background on reverse accumulation automatic differentiation.

  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).expr->adjointExpr = wpi::MakeIntrusiveShared<Expression>(0.0);
  }

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, wpi::IntrusiveSharedPtr<Expression>>> stack;
  stack.reserve(1024);

  stack.emplace_back(variable, wpi::MakeIntrusiveShared<Expression>(1.0));
  while (!stack.empty()) {
    Variable var = std::move(std::get<0>(stack.back()));
    wpi::IntrusiveSharedPtr<Expression> adjoint =
        std::move(std::get<1>(stack.back()));
    stack.pop_back();

    auto& lhs = var.expr->args[0];
    auto& rhs = var.expr->args[1];

    if (var.expr->adjointExpr == nullptr) {
      var.expr->adjointExpr = adjoint;
    } else {
      var.expr->adjointExpr = var.expr->adjointExpr + adjoint;
    }

    if (lhs != nullptr) {
      stack.emplace_back(lhs, adjoint * var.expr->gradientFuncs[0](lhs, rhs));

      if (rhs != nullptr) {
        stack.emplace_back(rhs, adjoint * var.expr->gradientFuncs[1](lhs, rhs));
      }
    }
  }

  VectorXvar grad{wrt.rows()};
  for (int row = 0; row < wrt.rows(); ++row) {
    grad(row) = Variable{wrt(row).expr->adjointExpr};
  }

  // Free adjoint storage that's no longer needed
  stack.emplace_back(variable, nullptr);
  while (!stack.empty()) {
    Variable var = std::move(std::get<0>(stack.back()));
    stack.pop_back();

    auto& lhs = var.expr->args[0];
    auto& rhs = var.expr->args[1];

    var.expr->adjointExpr = nullptr;

    if (lhs != nullptr) {
      stack.emplace_back(lhs, nullptr);

      if (rhs != nullptr) {
        stack.emplace_back(rhs, nullptr);
      }
    }
  }

  return grad;
}
