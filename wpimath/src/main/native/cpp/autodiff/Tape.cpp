// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Tape.h"

#include <array>
#include <utility>

#include "frc/autodiff/Expression.h"

using namespace frc::autodiff;

Variable Tape::PushNullary(double value, BinaryFuncDouble gradientValueFunc,
                           BinaryFuncVar gradientFunc) {
  m_expressions.emplace_back(value, std::move(gradientValueFunc),
                             std::move(gradientFunc));
  return Variable{static_cast<int>(m_expressions.size() - 1),
                  Variable::PrivateInit{}};
}

Variable Tape::PushUnary(Variable arg, BinaryFuncDouble valueFunc,
                         BinaryFuncDouble gradientValueFunc,
                         BinaryFuncVar gradientFunc) {
  m_expressions.emplace_back(
      std::array<Variable, Expression::kNumArgs>{arg, Variable{}},
      std::move(valueFunc),
      std::array<BinaryFuncDouble, Expression::kNumArgs>{
          std::move(gradientValueFunc), [](double, double) { return 0.0; }},
      std::array<BinaryFuncVar, Expression::kNumArgs>{
          std::move(gradientFunc),
          [](const Variable&, const Variable&) { return Constant(0.0); }});
  return Variable{static_cast<int>(m_expressions.size() - 1),
                  Variable::PrivateInit{}};
}

Variable Tape::PushBinary(Variable lhs, Variable rhs,
                          BinaryFuncDouble valueFunc,
                          BinaryFuncDouble lhsGradientValueFunc,
                          BinaryFuncVar lhsGradientFunc,
                          BinaryFuncDouble rhsGradientValueFunc,
                          BinaryFuncVar rhsGradientFunc) {
  m_expressions.emplace_back(
      std::array<Variable, Expression::kNumArgs>{lhs, rhs},
      std::move(valueFunc),
      std::array<BinaryFuncDouble, Expression::kNumArgs>{
          std::move(lhsGradientValueFunc), std::move(rhsGradientValueFunc)},
      std::array<BinaryFuncVar, Expression::kNumArgs>{
          std::move(lhsGradientFunc), std::move(rhsGradientFunc)});
  return Variable{static_cast<int>(m_expressions.size() - 1),
                  Variable::PrivateInit{}};
}

void Tape::Clear() {
  m_expressions.clear();
}

Expression& Tape::operator[](int pos) {
  return m_expressions[pos];
}

const Expression& Tape::operator[](int pos) const {
  return m_expressions[pos];
}

void Tape::Resize(int size) {
  m_expressions.erase(m_expressions.begin() + size, m_expressions.end());
}

int Tape::Size() const {
  return m_expressions.size();
}

Tape& Tape::GetTape() {
  static Tape tape;
  return tape;
}

Tape::Tape() {
  m_expressions.reserve(64000);
}
