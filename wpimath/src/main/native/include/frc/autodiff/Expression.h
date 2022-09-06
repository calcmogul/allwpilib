// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <variant>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

using UnaryFuncDouble = double (*)(double);
using BinaryFuncDouble = double (*)(double, double);
using VariantValueFunc =
    std::variant<std::monostate, UnaryFuncDouble, BinaryFuncDouble>;

using BinaryFuncVar = Variable (*)(const Variable&, const Variable&);

struct WPILIB_DLLEXPORT Expression {
  static constexpr int kNumArgs = 2;

  int index = 0;

  double value = 0.0;

  double adjoint = 0.0;

  // Indices of dependent nodes (function arguments)
  std::array<Variable, kNumArgs> args;

  // Either nullary operator with no arguments, unary operator with one
  // argument, or binary operator with two arguments. This operator is
  // used to update the node's value.
  VariantValueFunc valueFunc;

  // Gradients with respect to each argument
  std::array<BinaryFuncVar, kNumArgs> gradientFuncs;

  Expression(const Expression&) = default;
  Expression& operator=(const Expression&) = default;

  Expression(Expression&&) = default;
  Expression& operator=(Expression&&) = default;

  /**
   * Constructs a node with the given value.
   *
   * @param value The variable's value.
   * @param gradientFunc Gradient with respect to the variable.
   */
  Expression(double value, BinaryFuncVar gradientFunc);

  /**
   * Constructs a node with the given gradients, argument indices, and function
   * pointer to a binary operator between them.
   *
   * @param args Tape locations of binary operator's operands.
   * @param valueFunc Binary operator that produces this node's value.
   * @param gradientFuncs Gradients with respect to each operand.
   */
  Expression(std::array<Variable, kNumArgs> args, VariantValueFunc valueFunc,
             std::array<BinaryFuncVar, kNumArgs> gradientFuncs);

  /**
   * Returns gradient with respect to the given argument index.
   *
   * @param arg The argument index (0 to kNumArgs - 1).
   */
  Variable Gradient(int arg) const;

  /**
   * Update the value of this node based on the values of its dependent
   * nodes.
   */
  void Update();
};

}  // namespace frc::autodiff
