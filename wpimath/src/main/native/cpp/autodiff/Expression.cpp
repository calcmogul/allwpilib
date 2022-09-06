// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <utility>

using namespace frc::autodiff;

Expression::Expression(double value, BinaryFuncVar gradientFunc)
    : value{value},
      gradientFuncs{gradientFunc, [](const Variable&, const Variable&) {
                      return Constant(0.0);
                    }} {}

Expression::Expression(std::array<Variable, kNumArgs> args,
                       BinaryFuncDouble valueFunc,
                       std::array<BinaryFuncVar, kNumArgs> gradientFuncs)
    : args{std::move(args)},
      valueFunc{std::move(valueFunc)},
      gradientFuncs{std::move(gradientFuncs)} {
  if (this->args[0].index != -1) {
    if (this->args[1].index == -1) {
      auto& lhs = this->args[0].GetExpression();
      value = this->valueFunc(lhs.value, 0.0);
    } else {
      auto& lhs = this->args[0].GetExpression();
      auto& rhs = this->args[1].GetExpression();
      value = this->valueFunc(lhs.value, rhs.value);
    }
  }
}

Variable Expression::Gradient(int arg) const {
  return gradientFuncs[arg](args[0], args[1]);
}

void Expression::Update() {
  if (args[0].index != -1) {
    if (args[1].index == -1) {
      auto& lhs = args[0].GetExpression();
      lhs.Update();

      value = valueFunc(lhs.value, 0.0);
    } else {
      auto& lhs = args[0].GetExpression();
      lhs.Update();

      auto& rhs = args[1].GetExpression();
      rhs.Update();

      value = valueFunc(lhs.value, rhs.value);
    }
  }
}
