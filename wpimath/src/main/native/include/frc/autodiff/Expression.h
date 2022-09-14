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

  // Expression arguments
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

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
      double lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
      const SharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
      const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
      double lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
      const SharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
      const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
      double lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
      const SharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
      const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
      double lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
      const SharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
      const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
      const SharedPtr<Expression>& lhs);

  friend WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
      const SharedPtr<Expression>& lhs);

  /**
   * Update the value of this node based on the values of its dependent
   * nodes.
   */
  void Update();
};

/**
 * std::abs() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> abs(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::acos() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> acos(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::asin() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> asin(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::atan() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> atan(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::atan2() for Expressions.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> atan2(  // NOLINT
    const SharedPtr<Expression>& y, const SharedPtr<Expression>& x);

/**
 * std::cos() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> cos(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::cosh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> cosh(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::erf() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> erf(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::exp() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> exp(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::hypot() for Expressions.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> hypot(  // NOLINT
    const SharedPtr<Expression>& x, const SharedPtr<Expression>& y);

/**
 * std::log() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> log(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::log10() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> log10(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::pow() for Expressions.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> pow(  // NOLINT
    const SharedPtr<Expression>& base, const SharedPtr<Expression>& power);

/**
 * std::sin() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> sin(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::sinh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> sinh(const SharedPtr<Expression>& x);

/**
 * std::sqrt() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> sqrt(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::tan() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> tan(  // NOLINT
    const SharedPtr<Expression>& x);

/**
 * std::tanh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT SharedPtr<Expression> tanh(const SharedPtr<Expression>& x);

}  // namespace frc::autodiff
