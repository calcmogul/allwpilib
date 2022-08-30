// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "Eigen/Core"
#include "Eigen/SparseCore"

namespace frc::autodiff {

class WPILIB_DLLEXPORT Tape;
struct WPILIB_DLLEXPORT TapeNode;

class WPILIB_DLLEXPORT Variable {
 public:
  Tape* tape = nullptr;
  int index = 0;

  constexpr Variable() = default;

  Variable(const Variable&) = default;
  Variable& operator=(const Variable&) = default;

  Variable(Variable&&) = default;
  Variable& operator=(Variable&&) = default;

  Variable(double value);  // NOLINT

  Variable(int value);  // NOLINT

  /**
   * Constructs a Variable pointing to the specified entry on a tape.
   *
   * @param tape The tape the autodiff variable is on.
   * @param index The index of the autodiff variable on the tape.
   */
  explicit Variable(Tape* tape, int index);

  Variable& operator=(double value);

  Variable& operator=(int value);

  friend Variable operator*(double lhs, const Variable& rhs);

  friend Variable operator*(const Variable& lhs, double rhs);

  friend Variable operator*(const Variable& lhs, const Variable& rhs);

  Variable& operator*=(double rhs);

  Variable& operator*=(const Variable& rhs);

  friend Variable operator/(double lhs, const Variable& rhs);

  friend Variable operator/(const Variable& lhs, double rhs);

  friend Variable operator/(const Variable& lhs, const Variable& rhs);

  Variable& operator/=(double rhs);

  Variable& operator/=(const Variable& rhs);

  friend Variable operator+(double lhs, const Variable& rhs);

  friend Variable operator+(const Variable& lhs, double rhs);

  friend Variable operator+(const Variable& lhs, const Variable& rhs);

  Variable& operator+=(double rhs);

  Variable& operator+=(const Variable& rhs);

  friend Variable operator-(double lhs, const Variable& rhs);

  friend Variable operator-(const Variable& lhs, double rhs);

  friend Variable operator-(const Variable& lhs, const Variable& rhs);

  Variable& operator-=(double rhs);

  Variable& operator-=(const Variable& rhs);

  friend Variable operator-(const Variable& lhs);

  friend Variable operator+(const Variable& lhs);

  friend bool operator==(double lhs, const Variable& rhs);

  friend bool operator==(const Variable& lhs, double rhs);

  friend bool operator==(const Variable& lhs, const Variable& rhs);

  friend bool operator!=(double lhs, const Variable& rhs);

  friend bool operator!=(const Variable& lhs, double rhs);

  friend bool operator!=(const Variable& lhs, const Variable& rhs);

  friend bool operator<(double lhs, const Variable& rhs);

  friend bool operator<(const Variable& lhs, double rhs);

  friend bool operator<(const Variable& lhs, const Variable& rhs);

  friend bool operator>(double lhs, const Variable& rhs);

  friend bool operator>(const Variable& lhs, double rhs);

  friend bool operator>(const Variable& lhs, const Variable& rhs);

  friend bool operator<=(double lhs, const Variable& rhs);

  friend bool operator<=(const Variable& lhs, double rhs);

  friend bool operator<=(const Variable& lhs, const Variable& rhs);

  friend bool operator>=(double lhs, const Variable& rhs);

  friend bool operator>=(const Variable& lhs, double rhs);

  friend bool operator>=(const Variable& lhs, const Variable& rhs);

  /**
   * Returns the value of this variable.
   */
  double Value() const;

  /**
   * Returns gradient with respect to the given argument index.
   *
   * @param arg The argument index (0 to kNumDeps - 1).
   */
  Variable Gradient(int arg) const;

  /**
   * Update the value of this variable based on the values of its dependent
   * variables.
   */
  void Update();

  const TapeNode& GetNode() const;

  TapeNode& GetNode();
};

/**
 * Create a Variable from a constant that has a derivative of zero.
 */
WPILIB_DLLEXPORT Variable Constant(double value);

using VectorXvar = Eigen::Vector<frc::autodiff::Variable, Eigen::Dynamic>;

/**
 * Returns gradient of a variable with respect to the given variable.
 *
 * @param variable Variable of which to compute gradient.
 * @param wrt Variable with respect to which to compute the gradient.
 */
WPILIB_DLLEXPORT double Gradient(Variable variable, Variable& wrt);

/**
 * Returns gradient of a variable with respect to the given vector.
 *
 * @param variable Variable of which to compute the gradient.
 * @param wrt Vector with respect to which to compute the gradient.
 */
WPILIB_DLLEXPORT Eigen::VectorXd Gradient(Variable variable, VectorXvar& wrt);

/**
 * Returns the Jacobian of an autodiff vector with respect to the given vector.
 *
 * @param variables Vector of which to compute the Jacobian.
 * @param wrt Vector with respect to which to compute the Jacobian.
 */
WPILIB_DLLEXPORT Eigen::SparseMatrix<double> Jacobian(VectorXvar& variables,
                                                      VectorXvar& wrt);

/**
 * Returns the Hessian of an autodiff variable with respect to the given vector.
 *
 * @param variable Variable of which to compute the Hessian.
 * @param wrt Vector with respect to which to compute the Hessian.
 */
WPILIB_DLLEXPORT Eigen::SparseMatrix<double> Hessian(Variable variable,
                                                     VectorXvar& wrt);

/**
 * std::abs() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable abs(double x);  // NOLINT

/**
 * std::abs() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable abs(const Variable& x);  // NOLINT

/**
 * std::acos() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable acos(double x);  // NOLINT

/**
 * std::acos() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable acos(const Variable& x);  // NOLINT

/**
 * std::asin() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable asin(double x);  // NOLINT

/**
 * std::asin() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable asin(const Variable& x);  // NOLINT

/**
 * std::atan() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable atan(double x);  // NOLINT

/**
 * std::atan() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable atan(const Variable& x);  // NOLINT

/**
 * std::atan2() for Variables.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT Variable atan2(double y,  // NOLINT
                                const Variable& x);

/**
 * std::atan2() for Variables.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT Variable atan2(const Variable& y,  // NOLINT
                                double x);

/**
 * std::atan2() for Variables.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT Variable atan2(const Variable& y,  // NOLINT
                                const Variable& x);

/**
 * std::cos() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable cos(double x);  // NOLINT

/**
 * std::cos() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable cos(const Variable& x);  // NOLINT

/**
 * std::cosh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable cosh(double x);  // NOLINT

/**
 * std::cosh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable cosh(const Variable& x);  // NOLINT

/**
 * std::erf() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable erf(double x);  // NOLINT

/**
 * std::erf() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable erf(const Variable& x);  // NOLINT

/**
 * std::exp() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable exp(double x);  // NOLINT

/**
 * std::exp() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable exp(const Variable& x);  // NOLINT

/**
 * std::hypot() for Variables.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT Variable hypot(double x,  // NOLINT
                                const Variable& y);

/**
 * std::hypot() for Variables.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT Variable hypot(const Variable& x,  // NOLINT
                                double y);

/**
 * std::hypot() for Variables.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT Variable hypot(const Variable& x,  // NOLINT
                                const Variable& y);

/**
 * std::log() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable log(double x);  // NOLINT

/**
 * std::log() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable log(const Variable& x);  // NOLINT

/**
 * std::log10() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable log10(double x);  // NOLINT

/**
 * std::log10() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable log10(const Variable& x);  // NOLINT

/**
 * std::pow() for Variables.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT Variable pow(double base,  // NOLINT
                              const Variable& power);

/**
 * std::pow() for Variables.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT Variable pow(const Variable& base,  // NOLINT
                              double power);

/**
 * std::pow() for Variables.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT Variable pow(const Variable& base,  // NOLINT
                              const Variable& power);

/**
 * std::sin() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sin(double x);  // NOLINT

/**
 * std::sin() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sin(const Variable& x);  // NOLINT

/**
 * std::sinh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sinh(double x);  // NOLINT

/**
 * std::sinh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sinh(const Variable& x);  // NOLINT

/**
 * std::sqrt() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sqrt(double x);  // NOLINT

/**
 * std::sqrt() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable sqrt(const Variable& x);  // NOLINT

/**
 * std::tan() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable tan(double x);  // NOLINT

/**
 * std::tan() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable tan(const Variable& x);  // NOLINT

/**
 * std::tanh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable tanh(double x);  // NOLINT

/**
 * std::tanh() for Variables.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT Variable tanh(const Variable& x);  // NOLINT

}  // namespace frc::autodiff
