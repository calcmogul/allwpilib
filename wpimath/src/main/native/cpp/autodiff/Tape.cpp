// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Tape.h"

#include <array>
#include <utility>

using namespace frc::autodiff;

Tape::Tape() {
  m_nodes.reserve(64000);
}

Variable Tape::PushNullary(double value, VariantGradientFunc gradientFunc) {
  m_nodes.emplace_back(value, std::move(gradientFunc));
  m_nodes.back().index = m_nodes.size() - 1;
  return Variable{this, static_cast<int>(m_nodes.size() - 1)};
}

Variable Tape::PushUnary(Variable arg, VariantValueFunc valueFunc,
                         VariantGradientFunc gradientFunc) {
  m_nodes.emplace_back(
      std::array<Variable, TapeNode::kNumArgs>{arg, Variable{}},
      std::move(valueFunc),
      std::array<VariantGradientFunc, TapeNode::kNumArgs>{
          std::move(gradientFunc), []() -> Variable { return Constant(0.0); }});
  m_nodes.back().index = m_nodes.size() - 1;
  return Variable{this, static_cast<int>(m_nodes.size() - 1)};
}

Variable Tape::PushBinary(Variable lhs, Variable rhs,
                          VariantValueFunc valueFunc,
                          VariantGradientFunc lhsGradientFunc,
                          VariantGradientFunc rhsGradientFunc) {
  m_nodes.emplace_back(
      std::array<Variable, TapeNode::kNumArgs>{lhs, rhs}, std::move(valueFunc),
      std::array<VariantGradientFunc, TapeNode::kNumArgs>{
          std::move(lhsGradientFunc), std::move(rhsGradientFunc)});
  m_nodes.back().index = m_nodes.size() - 1;
  return Variable{this, static_cast<int>(m_nodes.size() - 1)};
}

void Tape::Clear() {
  m_nodes.clear();
}

TapeNode& Tape::operator[](int pos) {
  return m_nodes[pos];
}

const TapeNode& Tape::operator[](int pos) const {
  return m_nodes[pos];
}

void Tape::Resize(int size) {
  m_nodes.erase(m_nodes.begin() + size, m_nodes.end());
}

int Tape::Size() const {
  return m_nodes.size();
}

Tape& Tape::GetTape() {
  static Tape tape;
  return tape;
}
