// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <array>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/IntrusiveSharedPtr.h"

namespace frc::autodiff {

struct WPILIB_DLLEXPORT Expression {
  using BinaryFuncDouble = double (*)(double, double);
  using BinaryFuncExpr =
      IntrusiveSharedPtr<Expression> (*)(const IntrusiveSharedPtr<Expression>&,
                                         const IntrusiveSharedPtr<Expression>&);

  static constexpr int kNumArgs = 2;

  double value = 0.0;

  double adjoint = 0.0;

  IntrusiveSharedPtr<Expression> adjointExpr;

  // Either nullary operator with no arguments, unary operator with one
  // argument, or binary operator with two arguments. This operator is
  // used to update the node's value.
  BinaryFuncDouble valueFunc = [](double, double) { return 0.0; };

  // Gradients with respect to each argument
  std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs{
      [](double, double) { return 0.0; }, [](double, double) { return 0.0; }};

  // Gradients with respect to each argument
  std::array<BinaryFuncExpr, kNumArgs> gradientFuncs{
      [](const IntrusiveSharedPtr<Expression>&,
         const IntrusiveSharedPtr<Expression>&) {
        return MakeIntrusiveShared<Expression>(0.0);
      },
      [](const IntrusiveSharedPtr<Expression>&,
         const IntrusiveSharedPtr<Expression>&) {
        return MakeIntrusiveShared<Expression>(0.0);
      }};

  // Expression arguments
  std::array<IntrusiveSharedPtr<Expression>, kNumArgs> args{nullptr, nullptr};

  // Reference count for intrusive shared pointer
  uint32_t refCount = 0;

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
             BinaryFuncExpr lhsGradientFunc,
             IntrusiveSharedPtr<Expression> lhs);

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
             IntrusiveSharedPtr<Expression> lhs,
             IntrusiveSharedPtr<Expression> rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
      double lhs, const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
      const IntrusiveSharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
      const IntrusiveSharedPtr<Expression>& lhs,
      const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
      double lhs, const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
      const IntrusiveSharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
      const IntrusiveSharedPtr<Expression>& lhs,
      const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
      double lhs, const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
      const IntrusiveSharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
      const IntrusiveSharedPtr<Expression>& lhs,
      const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
      double lhs, const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
      const IntrusiveSharedPtr<Expression>& lhs, double rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
      const IntrusiveSharedPtr<Expression>& lhs,
      const IntrusiveSharedPtr<Expression>& rhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
      const IntrusiveSharedPtr<Expression>& lhs);

  friend WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
      const IntrusiveSharedPtr<Expression>& lhs);

  /**
   * Update the value of this node based on the values of its dependent
   * nodes.
   */
  void Update();
};

/**
 * Refcount increment for intrusive shared pointer.
 *
 * @param expr The shared pointer's managed object.
 */
inline void IntrusiveSharedPtrIncRefCount(Expression* expr) {
  ++expr->refCount;
}

/**
 * Refcount decrement for intrusive shared pointer.
 *
 * @param expr The shared pointer's managed object.
 */
inline void IntrusiveSharedPtrDecRefCount(Expression* expr) {
  if (--expr->refCount == 0) {
    delete expr;
  }
}

/**
 * std::abs() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> abs(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::acos() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> acos(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::asin() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> asin(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::atan() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> atan(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::atan2() for Expressions.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> atan2(  // NOLINT
    const IntrusiveSharedPtr<Expression>& y,
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::cos() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> cos(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::cosh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> cosh(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::erf() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> erf(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::exp() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> exp(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::hypot() for Expressions.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> hypot(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x,
    const IntrusiveSharedPtr<Expression>& y);

/**
 * std::log() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> log(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::log10() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> log10(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::pow() for Expressions.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> pow(  // NOLINT
    const IntrusiveSharedPtr<Expression>& base,
    const IntrusiveSharedPtr<Expression>& power);

/**
 * std::sin() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> sin(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::sinh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> sinh(
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::sqrt() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> sqrt(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::tan() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> tan(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x);

/**
 * std::tanh() for Expressions.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> tanh(
    const IntrusiveSharedPtr<Expression>& x);

}  // namespace frc::autodiff
