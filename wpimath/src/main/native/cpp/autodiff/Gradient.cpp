// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Gradient.h"

#include <tuple>
#include <vector>

namespace frc::autodiff {

double Gradient(Variable variable, Variable& wrt) {
  // Read wpimath/README.md#Reverse_accumulation_automatic_differentiation for
  // background on reverse accumulation automatic differentiation.

  wrt.expr->adjoint = 0.0;

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, double>> stack;
  stack.reserve(1024);

  stack.emplace_back(variable, 1.0);
  while (!stack.empty()) {
    Variable var = std::move(std::get<0>(stack.back()));
    double adjoint = std::move(std::get<1>(stack.back()));
    stack.pop_back();

    auto& lhs = var.expr->args[0];
    auto& rhs = var.expr->args[1];

    var.expr->adjoint += adjoint;

    if (lhs != nullptr) {
      if (rhs == nullptr) {
        stack.emplace_back(
            lhs, var.expr->gradientValueFuncs[0](lhs->value, 0.0, adjoint));
      } else {
        stack.emplace_back(lhs, var.expr->gradientValueFuncs[0](
                                              lhs->value, rhs->value, adjoint));
        stack.emplace_back(rhs, var.expr->gradientValueFuncs[1](
                                              lhs->value, rhs->value, adjoint));
      }
    }
  }

  return wrt.expr->adjoint;
}

Eigen::SparseVector<double> Gradient(Variable variable,
                                     Eigen::Ref<VectorXvar> wrt) {
  // Read wpimath/README.md#Reverse_accumulation_automatic_differentiation for
  // background on reverse accumulation automatic differentiation.

  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).expr->adjoint = 0.0;
  }

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, double>> stack;
  stack.reserve(1024);

  stack.emplace_back(variable, 1.0);
  while (!stack.empty()) {
    Variable var = std::move(std::get<0>(stack.back()));
    double adjoint = std::move(std::get<1>(stack.back()));
    stack.pop_back();

    auto& lhs = var.expr->args[0];
    auto& rhs = var.expr->args[1];

    var.expr->adjoint += adjoint;

    if (lhs != nullptr) {
      if (rhs == nullptr) {
        stack.emplace_back(
            lhs, adjoint * var.expr->gradientValueFuncs[0](lhs->value, 0.0));
      } else {
        stack.emplace_back(lhs, adjoint * var.expr->gradientValueFuncs[0](
                                              lhs->value, rhs->value));
        stack.emplace_back(rhs, adjoint * var.expr->gradientValueFuncs[1](
                                              lhs->value, rhs->value));
      }
    }
  }

  Eigen::SparseVector<double> grad{wrt.rows()};
  for (int row = 0; row < wrt.rows(); ++row) {
    double adjoint = wrt(row).expr->adjoint;
    if (adjoint != 0.0) {
      grad.insertBack(row) = adjoint;
    }
  }

  return grad;
}

}  // namespace frc::autodiff
