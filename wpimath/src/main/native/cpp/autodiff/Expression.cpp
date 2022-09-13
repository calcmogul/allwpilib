// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <utility>

using namespace frc::autodiff;

Expression::Expression(double value) : value{value} {}

Expression::Expression(BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       std::shared_ptr<Expression> lhs)
    : value{valueFunc(lhs->value, 0.0)},
      valueFunc{valueFunc},
      gradientValueFuncs{std::array{lhsGradientValueFunc, BinaryFuncDouble{}}},
      gradientFuncs{std::array{lhsGradientFunc, BinaryFuncExpr{}}},
      args{std::array<std::shared_ptr<Expression>, 2>{lhs, nullptr}} {}

Expression::Expression(BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncDouble rhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       BinaryFuncExpr rhsGradientFunc,
                       std::shared_ptr<Expression> lhs,
                       std::shared_ptr<Expression> rhs)
    : value{valueFunc(lhs->value, rhs->value)},
      valueFunc{valueFunc},
      gradientValueFuncs{
          std::array{lhsGradientValueFunc, rhsGradientValueFunc}},
      gradientFuncs{std::array{lhsGradientFunc, rhsGradientFunc}},
      args{std::array<std::shared_ptr<Expression>, 2>{lhs, rhs}} {}

void Expression::Update() {
  if (args[0] != nullptr) {
    auto& lhs = args[0];
    lhs->Update();

    if (args[1] == nullptr) {
      value = valueFunc(lhs->value, 0.0);
    } else {
      auto& rhs = args[1];
      rhs->Update();

      value = valueFunc(lhs->value, rhs->value);
    }
  }
}
