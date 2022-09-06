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
                       VariantValueFunc valueFunc,
                       std::array<BinaryFuncVar, kNumArgs> gradientFuncs)
    : args{std::move(args)},
      valueFunc{std::move(valueFunc)},
      gradientFuncs{std::move(gradientFuncs)} {
  if (std::holds_alternative<UnaryFuncDouble>(this->valueFunc)) {
    value = std::get<UnaryFuncDouble>(this->valueFunc)(
        this->args[0].GetExpression().value);
  } else if (std::holds_alternative<BinaryFuncDouble>(this->valueFunc)) {
    value = std::get<BinaryFuncDouble>(this->valueFunc)(
        this->args[0].GetExpression().value,
        this->args[1].GetExpression().value);
  }
}

Variable Expression::Gradient(int arg) const {
  return gradientFuncs[arg](args[0], args[1]);
}

void Expression::Update() {
  if (std::holds_alternative<UnaryFuncDouble>(valueFunc)) {
    auto& lhs = args[0].GetExpression();
    lhs.Update();

    value = std::get<UnaryFuncDouble>(valueFunc)(lhs.value);
  } else if (std::holds_alternative<BinaryFuncDouble>(valueFunc)) {
    auto& lhs = args[0].GetExpression();
    lhs.Update();

    auto& rhs = args[1].GetExpression();
    rhs.Update();

    value = std::get<BinaryFuncDouble>(valueFunc)(lhs.value, rhs.value);
  } else {
    return;
  }
}
