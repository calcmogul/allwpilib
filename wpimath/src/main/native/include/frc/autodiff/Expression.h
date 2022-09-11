// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

using BinaryFuncDouble = double (*)(double, double);
using BinaryFuncVar = Variable (*)(const Variable&, const Variable&);

struct WPILIB_DLLEXPORT Expression {
  static constexpr int kNumArgs = 2;

  double value = 0.0;

  double adjoint = 0.0;

  Variable adjointVar;

  // Indices of dependent nodes (function arguments)
  std::array<Variable, kNumArgs> args{Variable{}, Variable{}};

  // Either nullary operator with no arguments, unary operator with one
  // argument, or binary operator with two arguments. This operator is
  // used to update the node's value.
  BinaryFuncDouble valueFunc = [](double, double) { return 0.0; };

  // Gradients with respect to each argument
  std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs{
      [](double, double) { return 0.0; }, [](double, double) { return 0.0; }};

  // Gradients with respect to each argument
  std::array<BinaryFuncVar, kNumArgs> gradientFuncs{
      [](const Variable&, const Variable&) { return Constant(0.0); },
      [](const Variable&, const Variable&) { return Constant(0.0); }};

  Expression(const Expression&) = default;
  Expression& operator=(const Expression&) = default;

  Expression(Expression&&) = default;
  Expression& operator=(Expression&&) = default;

  /**
   * Constructs a node with the given gradients, argument indices, and function
   * pointer to a binary operator between them.
   *
   * @param value This node's value.
   * @param valueFunc Binary operator that produces this node's value.
   * @param gradientValueFuncs Gradients with respect to each operand.
   * @param gradientFuncs Gradients with respect to each operand.
   * @param args Tape locations of binary operator's operands.
   */
  Expression(double value, BinaryFuncDouble valueFunc,
             std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs,
             std::array<BinaryFuncVar, kNumArgs> gradientFuncs,
             std::array<Variable, kNumArgs> args);

  /**
   * Update the value of this node based on the values of its dependent
   * nodes.
   */
  void Update();
};

}  // namespace frc::autodiff
