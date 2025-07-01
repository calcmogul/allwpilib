// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import edu.wpi.first.math.jni.WPIMathJNI;

/** Variable JNI functions. */
public class VariableJNI extends WPIMathJNI {
  /**
   * Constructs a Variable from a floating point type.
   *
   * @param value The value of the Variable.
   */
  static native long createDouble(double value);

  /**
   * Constructs a Variable from an integral type.
   *
   * @param value The value of the Variable.
   */
  static native long createInt(int value);

  /**
   * Destructs a Variable.
   *
   * @param handle Variable handle.
   */
  static native void destroy(long handle);

  /**
   * Sets Variable's internal value.
   *
   * @param handle Variable handle.
   * @param value The value of the Variable.
   */
  static native void setValue(long handle, double value);

  /**
   * Variable-Variable multiplication operator.
   *
   * @param handle Variable handle.
   * @param rhs Operator right-hand side.
   * @return Result of multiplication.
   */
  static native long times(long handle, long rhs);

  /**
   * Variable-Variable division operator.
   *
   * @param handle Variable handle.
   * @param rhs Operator right-hand side.
   * @return Result of division.
   */
  static native long div(long handle, long rhs);

  /**
   * Variable-Variable addition operator.
   *
   * @param handle Variable handle.
   * @param rhs Operator right-hand side.
   * @return Result of addition.
   */
  static native long plus(long handle, long rhs);

  /**
   * Variable-Variable subtraction operator.
   *
   * @param handle Variable handle.
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  static native long minus(long handle, long rhs);

  /**
   * Unary minus operator.
   *
   * @param handle Variable handle.
   */
  static native long unaryMinus(long handle);

  /**
   * Returns the value of this variable.
   *
   * @param handle Variable handle.
   * @return The value of this variable.
   */
  static native double value(long handle);

  /**
   * Returns the type of this expression (constant, linear, quadratic, or nonlinear).
   *
   * @param handle Variable handle.
   * @return The type of this expression.
   */
  static native ExpressionType type(long handle);

  /**
   * std::abs() for Variables.
   *
   * @param x The argument.
   */
  static native long abs(long x);

  /**
   * std::acos() for Variables.
   *
   * @param x The argument.
   */
  static native long acos(long x);

  /**
   * std::asin() for Variables.
   *
   * @param x The argument.
   */
  static native long asin(long x);

  /**
   * std::atan() for Variables.
   *
   * @param x The argument.
   */
  static native long atan(long x);

  /**
   * std::atan2() for Variables.
   *
   * @param y The y argument.
   * @param x The x argument.
   */
  static native long atan2(long y, long x);

  /**
   * std::cos() for Variables.
   *
   * @param x The argument.
   */
  static native long cos(long x);

  /**
   * std::cosh() for Variables.
   *
   * @param x The argument.
   */
  static native long cosh(long x);

  /**
   * std::erf() for Variables.
   *
   * @param x The argument.
   */
  static native long erf(long x);

  /**
   * std::exp() for Variables.
   *
   * @param x The argument.
   */
  static native long exp(long x);

  /**
   * std::hypot() for Variables.
   *
   * @param x The x argument.
   * @param y The y argument.
   */
  static native long hypot(long x, long y);

  /**
   * std::pow() for Variables.
   *
   * @param base The base.
   * @param power The power.
   */
  static native long pow(long base, long power);

  /**
   * std::log() for Variables.
   *
   * @param x The argument.
   */
  static native long log(long x);

  /**
   * std::log10() for Variables.
   *
   * @param x The argument.
   */
  static native long log10(long x);

  /**
   * sign() for Variables.
   *
   * @param x The argument.
   */
  static native long sign(long x);

  /**
   * std::sin() for Variables.
   *
   * @param x The argument.
   */
  static native long sin(long x);

  /**
   * std::sinh() for Variables.
   *
   * @param x The argument.
   */
  static native long sinh(long x);

  /**
   * std::sqrt() for Variables.
   *
   * @param x The argument.
   */
  static native long sqrt(long x);

  /**
   * std::tan() for Variables.
   *
   * @param x The argument.
   */
  static native long tan(long x);

  /**
   * std::tanh() for Variables.
   *
   * @param x The argument.
   */
  static native long tanh(long x);
}
