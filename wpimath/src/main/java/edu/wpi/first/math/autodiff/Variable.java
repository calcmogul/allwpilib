// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

/** An autodiff variable pointing to an expression node. */
public class Variable implements AutoCloseable {
  private long m_handle;

  /** Constructs a linear Variable with a value of zero. */
  public Variable() {}

  /**
   * Constructs a Variable from a floating point type.
   *
   * @param value The value of the Variable.
   */
  public Variable(double value) {
    m_handle = VariableJNI.createDouble(value);
  }

  /**
   * Constructs a Variable from an integral type.
   *
   * @param value The value of the Variable.
   */
  public Variable(int value) {
    m_handle = VariableJNI.createInt(value);
  }

  @Override
  public void close() {
    VariableJNI.destroy(m_handle);
  }

  /**
   * Sets Variable's internal value.
   *
   * @param value The value of the Variable.
   */
  public void setValue(double value) {
    VariableJNI.setValue(m_handle, value);
  }

  /**
   * Variable-Variable multiplication operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of multiplication.
   */
  public Variable times(Variable rhs) {
    return fromHandle(VariableJNI.times(m_handle, rhs.getHandle()));
  }

  /**
   * Variable-Variable division operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of division.
   */
  public Variable div(Variable rhs) {
    return fromHandle(VariableJNI.div(m_handle, rhs.getHandle()));
  }

  /**
   * Variable-Variable addition operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of addition.
   */
  public Variable plus(Variable rhs) {
    return fromHandle(VariableJNI.plus(m_handle, rhs.getHandle()));
  }

  /**
   * Variable-Variable subtraction operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  public Variable minus(Variable rhs) {
    return fromHandle(VariableJNI.minus(m_handle, rhs.getHandle()));
  }

  /**
   * Variable-Variable subtraction operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  public Variable minus(double rhs) {
    return fromHandle(VariableJNI.minus(m_handle, new Variable(rhs).getHandle()));
  }

  /** Unary minus operator. */
  public Variable unaryMinus() {
    return fromHandle(VariableJNI.unaryMinus(m_handle));
  }

  /**
   * Returns the value of this variable.
   *
   * @return The value of this variable.
   */
  public double value() {
    return VariableJNI.value(m_handle);
  }

  /**
   * Returns the type of this expression (constant, linear, quadratic, or nonlinear).
   *
   * @return The type of this expression.
   */
  public ExpressionType type() {
    return VariableJNI.type(m_handle);
  }

  /**
   * Returns internal handle.
   *
   * @return Internal handle.
   */
  public long getHandle() {
    return m_handle;
  }

  /**
   * Constructs a Variable from an internal handle.
   *
   * <p>This should be used by JNI only.
   *
   * @param handle Variable handle.
   */
  public static Variable fromHandle(long handle) {
    var result = new Variable();
    result.m_handle = handle;
    return result;
  }

  /**
   * std::abs() for Variables.
   *
   * @param x The argument.
   */
  public static Variable abs(Variable x) {
    return fromHandle(VariableJNI.abs(x.getHandle()));
  }

  /**
   * std::acos() for Variables.
   *
   * @param x The argument.
   */
  public static Variable acos(Variable x) {
    return fromHandle(VariableJNI.acos(x.getHandle()));
  }

  /**
   * std::asin() for Variables.
   *
   * @param x The argument.
   */
  public static Variable asin(Variable x) {
    return fromHandle(VariableJNI.asin(x.getHandle()));
  }

  /**
   * std::atan() for Variables.
   *
   * @param x The argument.
   */
  public static Variable atan(Variable x) {
    return fromHandle(VariableJNI.atan(x.getHandle()));
  }

  /**
   * std::atan2() for Variables.
   *
   * @param y The y argument.
   * @param x The x argument.
   */
  public static Variable atan2(Variable y, Variable x) {
    return fromHandle(VariableJNI.atan2(y.getHandle(), x.getHandle()));
  }

  /**
   * std::cos() for Variables.
   *
   * @param x The argument.
   */
  public static Variable cos(Variable x) {
    return fromHandle(VariableJNI.cos(x.getHandle()));
  }

  /**
   * std::cosh() for Variables.
   *
   * @param x The argument.
   */
  public static Variable cosh(Variable x) {
    return fromHandle(VariableJNI.cosh(x.getHandle()));
  }

  /**
   * std::erf() for Variables.
   *
   * @param x The argument.
   */
  public static Variable erf(Variable x) {
    return fromHandle(VariableJNI.erf(x.getHandle()));
  }

  /**
   * std::exp() for Variables.
   *
   * @param x The argument.
   */
  public static Variable exp(Variable x) {
    return fromHandle(VariableJNI.exp(x.getHandle()));
  }

  /**
   * std::hypot() for Variables.
   *
   * @param x The x argument.
   * @param y The y argument.
   */
  public static Variable hypot(Variable x, Variable y) {
    return fromHandle(VariableJNI.hypot(x.getHandle(), y.getHandle()));
  }

  /**
   * std::pow() for Variables.
   *
   * @param base The base.
   * @param power The power.
   */
  public static Variable pow(Variable base, Variable power) {
    return fromHandle(VariableJNI.pow(base.getHandle(), power.getHandle()));
  }

  /**
   * std::pow() for Variables.
   *
   * @param base The base.
   * @param power The power.
   */
  public static Variable pow(Variable base, int power) {
    return fromHandle(VariableJNI.pow(base.getHandle(), new Variable(power).getHandle()));
  }

  /**
   * std::log() for Variables.
   *
   * @param x The argument.
   */
  public static Variable log(Variable x) {
    return fromHandle(VariableJNI.log(x.getHandle()));
  }

  /**
   * std::log10() for Variables.
   *
   * @param x The argument.
   */
  public static Variable log10(Variable x) {
    return fromHandle(VariableJNI.log10(x.getHandle()));
  }

  /**
   * sign() for Variables.
   *
   * @param x The argument.
   */
  public static Variable sign(Variable x) {
    return fromHandle(VariableJNI.sign(x.getHandle()));
  }

  /**
   * std::sin() for Variables.
   *
   * @param x The argument.
   */
  public static Variable sin(Variable x) {
    return fromHandle(VariableJNI.sin(x.getHandle()));
  }

  /**
   * std::sinh() for Variables.
   *
   * @param x The argument.
   */
  public static Variable sinh(Variable x) {
    return fromHandle(VariableJNI.sinh(x.getHandle()));
  }

  /**
   * std::sqrt() for Variables.
   *
   * @param x The argument.
   */
  public static Variable sqrt(Variable x) {
    return fromHandle(VariableJNI.sqrt(x.getHandle()));
  }

  /**
   * std::tan() for Variables.
   *
   * @param x The argument.
   */
  public static Variable tan(Variable x) {
    return fromHandle(VariableJNI.tan(x.getHandle()));
  }

  /**
   * std::tanh() for Variables.
   *
   * @param x The argument.
   */
  public static Variable tanh(Variable x) {
    return fromHandle(VariableJNI.tanh(x.getHandle()));
  }

  /**
   * std::hypot() for Variables.
   *
   * @param x The x argument.
   * @param y The y argument.
   * @param z The z argument.
   */
  public static Variable hypot(Variable x, Variable y, Variable z) {
    return sqrt(pow(x, 2).plus(pow(y, 2)).plus(pow(z, 2)));
  }
}
