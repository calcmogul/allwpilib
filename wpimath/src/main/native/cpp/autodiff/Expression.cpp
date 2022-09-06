// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <utility>

using namespace frc::autodiff;

Expression::Expression(double value, BinaryFuncDouble gradientValueFunc,
                       BinaryFuncVar gradientFunc)
    : value{value},
      gradientValueFuncs{gradientValueFunc, [](double, double) { return 0.0; }},
      gradientFuncs{gradientFunc, [](const Variable&,
                                     const Variable&) { return Constant(0.0); }}

{}

Expression::Expression(
    std::array<Variable, kNumArgs> args, BinaryFuncDouble valueFunc,
    std::array<BinaryFuncDouble, kNumArgs> gradientValueFuncs,
    std::array<BinaryFuncVar, kNumArgs> gradientFuncs)
    : args{std::move(args)},
      valueFunc{std::move(valueFunc)},
      gradientValueFuncs{std::move(gradientValueFuncs)},
      gradientFuncs{std::move(gradientFuncs)} {
  if (this->args[0].index != -1) {
    auto& lhs = this->args[0].GetExpression();

    if (this->args[1].index == -1) {
      value = this->valueFunc(lhs.value, 0.0);
    } else {
      auto& rhs = this->args[1].GetExpression();

      value = this->valueFunc(lhs.value, rhs.value);
    }
  }
}

double Expression::GradientValue(int arg) const {
  return gradientValueFuncs[arg](args[0].Value(), args[1].Value());
}

Variable Expression::Gradient(int arg) const {
  return gradientFuncs[arg](args[0], args[1]);
}

void Expression::Update() {
  if (args[0].index != -1) {
    auto& lhs = args[0].GetExpression();
    lhs.Update();

    if (args[1].index == -1) {
      value = valueFunc(lhs.value, 0.0);
    } else {
      auto& rhs = args[1].GetExpression();
      rhs.Update();

      value = valueFunc(lhs.value, rhs.value);
    }
  }
}
