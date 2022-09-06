// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/TapeNode.h"
#include "frc/autodiff/Variable.h"

namespace frc::autodiff {

class WPILIB_DLLEXPORT Tape {
 public:
  Tape();

  /**
   * Pushes a new node with no arguments and returns its location.
   *
   * @param value The node value.
   * @param gradientFunc Gradient with respect to variable.
   */
  Variable PushNullary(double value, VariantGradientFunc gradientFunc);

  /**
   * Pushes a new node with one argument and returns its location.
   *
   * @param arg Unary operator's operand.
   * @param valueFunc Unary operator that produces this node's value.
   * @param gradientFunc Gradient with respect to operand.
   */
  Variable PushUnary(Variable arg, VariantValueFunc valueFunc,
                     VariantGradientFunc gradientFunc);

  /**
   * Pushes a new node with two arguments and returns its location.
   *
   * @param lhs Binary operator's left operand.
   * @param rhs Binary operator's right operand.
   * @param valueFunc Binary operator that produces this node's value.
   * @param lhsGradientFunc Gradient with respect to left operand.
   * @param rhsGradientFunc Gradient with respect to right operand.
   */
  Variable PushBinary(Variable lhs, Variable rhs, VariantValueFunc valueFunc,
                      VariantGradientFunc lhsGradientFunc,
                      VariantGradientFunc rhsGradientFunc);

  /**
   * Clear the tape. All Variables pointing to the tape will become invalid.
   */
  void Clear();

  TapeNode& operator[](int pos);

  const TapeNode& operator[](int pos) const;

  /**
   * Resize the tape.
   *
   * @param size The new tape size.
   */
  void Resize(int size);

  /**
   * Returns the number of elements on the tape.
   */
  int Size() const;

  /**
   * Returns the global tape.
   */
  static Tape& GetTape();

 private:
  std::vector<TapeNode> m_nodes;
};

}  // namespace frc::autodiff
