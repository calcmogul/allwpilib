// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <utility>

using namespace frc::autodiff;

Expression::Expression(
    double value, BinaryFuncDouble valueFunc,
    std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs,
    std::array<BinaryFuncVar, kNumArgs> gradientFuncs,
    std::array<Variable, kNumArgs> args)
    : value{value},
      args{std::move(args)},
      valueFunc{std::move(valueFunc)},
      gradientValueFuncs{std::move(gradientValueFuncs)},
      gradientFuncs{std::move(gradientFuncs)} {}

void Expression::Update() {
  if (args[0].expr != nullptr) {
    auto& lhs = args[0].GetExpression();
    lhs.Update();

    if (args[1].expr == nullptr) {
      value = valueFunc(lhs.value, 0.0);
    } else {
      auto& rhs = args[1].GetExpression();
      rhs.Update();

      value = valueFunc(lhs.value, rhs.value);
    }
  }
}
