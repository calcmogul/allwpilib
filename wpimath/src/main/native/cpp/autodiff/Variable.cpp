// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Variable.h"

#include <cmath>
#include <tuple>
#include <vector>

#include <wpi/SymbolExports.h>

namespace frc::autodiff {

Variable::Variable(double value)
    : expr{wpi::MakeIntrusiveShared<Expression>(value)} {}

Variable::Variable(int value)
    : expr{wpi::MakeIntrusiveShared<Expression>(value)} {}

Variable::Variable(wpi::IntrusiveSharedPtr<Expression> expr)
    : expr{std::move(expr)} {}

Variable& Variable::operator=(double value) {
  if (expr == nullptr) {
    expr = wpi::MakeIntrusiveShared<Expression>(value);
  } else {
    GetExpression().value = value;
  }
  return *this;
}

Variable& Variable::operator=(int value) {
  if (expr == nullptr) {
    expr = wpi::MakeIntrusiveShared<Expression>(value);
  } else {
    GetExpression().value = value;
  }
  return *this;
}

WPILIB_DLLEXPORT Variable operator*(double lhs, const Variable& rhs) {
  return Variable{lhs} * rhs;
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, double rhs) {
  return lhs * Variable{rhs};
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, const Variable& rhs) {
  return Variable{lhs.expr * rhs.expr};
}

Variable& Variable::operator*=(double rhs) {
  *this = *this * rhs;
  return *this;
}

Variable& Variable::operator*=(const Variable& rhs) {
  *this = *this * rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator/(double lhs, const Variable& rhs) {
  return Variable{lhs} / rhs;
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, double rhs) {
  return lhs / Variable{rhs};
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, const Variable& rhs) {
  return Variable{lhs.expr / rhs.expr};
}

Variable& Variable::operator/=(double rhs) {
  *this = *this / rhs;
  return *this;
}

Variable& Variable::operator/=(const Variable& rhs) {
  *this = *this / rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator+(double lhs, const Variable& rhs) {
  return Variable{lhs} + rhs;
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, double rhs) {
  return lhs + Variable{rhs};
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, const Variable& rhs) {
  return Variable{lhs.expr + rhs.expr};
}

Variable& Variable::operator+=(double rhs) {
  *this = *this + rhs;
  return *this;
}

Variable& Variable::operator+=(const Variable& rhs) {
  *this = *this + rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator-(double lhs, const Variable& rhs) {
  return Variable{lhs} - rhs;
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, double rhs) {
  return lhs - Variable{rhs};
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, const Variable& rhs) {
  return Variable{lhs.expr - rhs.expr};
}

Variable& Variable::operator-=(double rhs) {
  *this = *this - rhs;
  return *this;
}

Variable& Variable::operator-=(const Variable& rhs) {
  *this = *this - rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs) {
  return Variable{-lhs.expr};
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs) {
  return Variable{+lhs.expr};
}

WPILIB_DLLEXPORT bool operator==(double lhs, const Variable& rhs) {
  return std::abs(lhs - rhs.Value()) < 1e-9;
}

WPILIB_DLLEXPORT bool operator==(const Variable& lhs, double rhs) {
  return std::abs(lhs.Value() - rhs) < 1e-9;
}

WPILIB_DLLEXPORT bool operator==(const Variable& lhs, const Variable& rhs) {
  return std::abs(lhs.Value() - rhs.Value()) < 1e-9;
}

WPILIB_DLLEXPORT bool operator!=(double lhs, const Variable& rhs) {
  return lhs != rhs.Value();
}

WPILIB_DLLEXPORT bool operator!=(const Variable& lhs, double rhs) {
  return lhs.Value() != rhs;
}

WPILIB_DLLEXPORT bool operator!=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() != rhs.Value();
}

WPILIB_DLLEXPORT bool operator<(double lhs, const Variable& rhs) {
  return lhs < rhs.Value();
}

WPILIB_DLLEXPORT bool operator<(const Variable& lhs, double rhs) {
  return lhs.Value() < rhs;
}

WPILIB_DLLEXPORT bool operator<(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() < rhs.Value();
}

WPILIB_DLLEXPORT bool operator>(double lhs, const Variable& rhs) {
  return lhs > rhs.Value();
}

WPILIB_DLLEXPORT bool operator>(const Variable& lhs, double rhs) {
  return lhs.Value() > rhs;
}

WPILIB_DLLEXPORT bool operator>(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() > rhs.Value();
}

WPILIB_DLLEXPORT bool operator<=(double lhs, const Variable& rhs) {
  return lhs <= rhs.Value();
}

WPILIB_DLLEXPORT bool operator<=(const Variable& lhs, double rhs) {
  return lhs.Value() <= rhs;
}

WPILIB_DLLEXPORT bool operator<=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() <= rhs.Value();
}

WPILIB_DLLEXPORT bool operator>=(double lhs, const Variable& rhs) {
  return lhs >= rhs.Value();
}

WPILIB_DLLEXPORT bool operator>=(const Variable& lhs, double rhs) {
  return lhs.Value() >= rhs;
}

WPILIB_DLLEXPORT bool operator>=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() >= rhs.Value();
}

double Variable::Value() const {
  if (expr == nullptr) {
    return 0.0;
  } else {
    return expr->value;
  }
}

void Variable::Update() {
  expr->Update();
}

const Expression& Variable::GetExpression() const {
  return *expr;
}

Expression& Variable::GetExpression() {
  return *expr;
}

Variable abs(double x) {
  return autodiff::abs(Variable{x});
}

Variable abs(const Variable& x) {
  return Variable{abs(x.expr)};
}

Variable acos(double x) {
  return autodiff::acos(Variable{x});
}

Variable acos(const Variable& x) {
  return Variable{acos(x.expr)};
}

Variable asin(double x) {
  return autodiff::asin(Variable{x});
}

Variable asin(const Variable& x) {
  return Variable{asin(x.expr)};
}

Variable atan(double x) {
  return autodiff::atan(Variable{x});
}

Variable atan(const Variable& x) {
  return Variable{atan(x.expr)};
}

Variable atan2(double y, const Variable& x) {
  return autodiff::atan2(Variable{y}, x);
}

Variable atan2(const Variable& y, double x) {
  return autodiff::atan2(y, Variable{x});
}

Variable atan2(const Variable& y, const Variable& x) {
  return Variable{atan2(y.expr, x.expr)};
}

Variable cos(double x) {
  return autodiff::cos(Variable{x});
}

Variable cos(const Variable& x) {
  return Variable{cos(x.expr)};
}

Variable cosh(double x) {
  return autodiff::cosh(Variable{x});
}

Variable cosh(const Variable& x) {
  return Variable{cosh(x.expr)};
}

Variable erf(double x) {
  return autodiff::erf(Variable{x});
}

Variable erf(const Variable& x) {
  return Variable{erf(x.expr)};
}

Variable exp(double x) {
  return autodiff::exp(Variable{x});
}

Variable exp(const Variable& x) {
  return Variable{exp(x.expr)};
}

Variable hypot(double x, const Variable& y) {
  return autodiff::hypot(Variable{x}, y);
}

Variable hypot(const Variable& x, double y) {
  return autodiff::hypot(x, Variable{y});
}

Variable hypot(const Variable& x, const Variable& y) {
  return Variable{hypot(x.expr, y.expr)};
}

Variable log(double x) {
  return autodiff::log(Variable{x});
}

Variable log(const Variable& x) {
  return Variable{log(x.expr)};
}

Variable log10(double x) {
  return autodiff::log10(Variable{x});
}

Variable log10(const Variable& x) {
  return Variable{log10(x.expr)};
}

Variable pow(double base, const Variable& power) {
  return autodiff::pow(Variable{base}, power);
}

Variable pow(const Variable& base, double power) {
  return autodiff::pow(base, Variable{power});
}

Variable pow(const Variable& base, const Variable& power) {
  return Variable{pow(base.expr, power.expr)};
}

Variable sin(double x) {
  return autodiff::sin(Variable{x});
}

Variable sin(const Variable& x) {
  return Variable{sin(x.expr)};
}

Variable sinh(double x) {
  return autodiff::sinh(Variable{x});
}

Variable sinh(const Variable& x) {
  return Variable{sinh(x.expr)};
}

Variable sqrt(double x) {
  return autodiff::sqrt(Variable{x});
}

Variable sqrt(const Variable& x) {
  return Variable{sqrt(x.expr)};
}

Variable tan(double x) {
  return autodiff::tan(Variable{x});
}

Variable tan(const Variable& x) {
  return Variable{tan(x.expr)};
}

Variable tanh(double x) {
  return autodiff::tanh(Variable{x});
}

Variable tanh(const Variable& x) {
  return Variable{tanh(x.expr)};
}

}  // namespace frc::autodiff
