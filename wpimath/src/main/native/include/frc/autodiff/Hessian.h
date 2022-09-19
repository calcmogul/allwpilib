// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/SymbolExports.h>

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

/**
 * This class caches the gradient tree of a variable with respect to a vector of
 * variables so subsequent Hessian calculations are faster.
 */
class WPILIB_DLLEXPORT Hessian {
 public:
  /**
   * Constructs a Hessian object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Variables with respect to which to compute the gradient.
   */
  Hessian(Variable variable, Eigen::Ref<VectorXvar> wrt);

  /**
   * Calculate the Hessian.
   */
  Eigen::SparseMatrix<double> Calculate();

 private:
  Variable m_variable;
  VectorXvar m_wrt;

  VectorXvar m_gradientTree;

  std::vector<Eigen::Triplet<double>> m_triplets;

  /**
   * Returns the given variable's gradient tree.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Variables with respect to which to compute the gradient.
   */
  static VectorXvar GenerateGradientTree(Variable& variable, VectorXvar& wrt);
};

}  // namespace frc::autodiff
