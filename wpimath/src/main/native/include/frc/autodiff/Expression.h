// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/SharedPtr.h"

namespace frc::autodiff {

struct WPILIB_DLLEXPORT Expression {
  using BinaryFuncDouble = double (*)(double, double);
  using BinaryFuncExpr = SharedPtr<Expression> (*)(
      const SharedPtr<Expression>&, const SharedPtr<Expression>&);

  static constexpr int kNumArgs = 2;

  double value = 0.0;

  double adjoint = 0.0;

  SharedPtr<Expression> adjointExpr;

  // Either nullary operator with no arguments, unary operator with one
  // argument, or binary operator with two arguments. This operator is
  // used to update the node's value.
  BinaryFuncDouble valueFunc = [](double, double) { return 0.0; };

  // Gradients with respect to each argument
  std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs{
      [](double, double) { return 0.0; }, [](double, double) { return 0.0; }};

  // Gradients with respect to each argument
  std::array<BinaryFuncExpr, kNumArgs> gradientFuncs{
      [](const SharedPtr<Expression>&, const SharedPtr<Expression>&) {
        return MakeShared<Expression>(0.0);
      },
      [](const SharedPtr<Expression>&, const SharedPtr<Expression>&) {
        return MakeShared<Expression>(0.0);
      }};

  // Indices of dependent nodes (function arguments)
  std::array<SharedPtr<Expression>, kNumArgs> args{nullptr, nullptr};

  Expression(const Expression&) = default;
  Expression& operator=(const Expression&) = default;

  Expression(Expression&&) = default;
  Expression& operator=(Expression&&) = default;

  /**
   * Constructs a nullary expression (an operator with no arguments).
   *
   * @param value The expression value.
   */
  explicit Expression(double value);

  /**
   * Constructs an unary expression (an operator with one argument).
   *
   * @param valueFunc Unary operator that produces this expression's value.
   * @param lhsGradientValueFunc Gradient with respect to the operand.
   * @param lhsGradientFunc Gradient with respect to the operand.
   * @param lhs Unary operator's operand.
   */
  Expression(BinaryFuncDouble valueFunc, BinaryFuncDouble lhsGradientValueFunc,
             BinaryFuncExpr lhsGradientFunc, SharedPtr<Expression> lhs);

  /**
   * Constructs a binary expression (an operator with two arguments).
   *
   * @param valueFunc Unary operator that produces this expression's value.
   * @param lhsGradientValueFunc Gradient with respect to the left operand.
   * @param rhsGradientValueFunc Gradient with respect to the right operand.
   * @param lhsGradientFunc Gradient with respect to the left operand.
   * @param rhsGradientFunc Gradient with respect to the right operand.
   * @param lhs Binary operator's left operand.
   * @param rhs Binary operator's right operand.
   */
  Expression(BinaryFuncDouble valueFunc, BinaryFuncDouble lhsGradientValueFunc,
             BinaryFuncDouble rhsGradientValueFunc,
             BinaryFuncExpr lhsGradientFunc, BinaryFuncExpr rhsGradientFunc,
             SharedPtr<Expression> lhs, SharedPtr<Expression> rhs);

  /**
   * Update the value of this node based on the values of its dependent
   * nodes.
   */
  void Update();
};

}  // namespace frc::autodiff
