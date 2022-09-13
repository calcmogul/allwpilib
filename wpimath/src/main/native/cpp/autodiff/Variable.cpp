// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Variable.h"

#include <cmath>
#include <tuple>
#include <vector>

#include <wpi/SymbolExports.h>

namespace frc::autodiff {

namespace {

/**
 * In reverse accumulation AD, the dependent variable to be differentiated is
 * fixed and the derivative is computed with respect to each sub-expression
 * recursively. In a pen-and-paper calculation, the derivative of the outer
 * functions is repeatedly substituted in the chain rule:
 *
 * (∂y/∂x) = (∂y/∂w₁) * (∂w₁/∂x) = ((∂y/∂w₂) * (∂w₂/∂w₁)) * (∂w₁/∂x) = ...
 *
 * In reverse accumulation, the quantity of interest is the adjoint,
 * denoted with a bar (w̄); it is a derivative of a chosen dependent variable
 * with respect to a subexpression w: ∂y/∂w.
 *
 * Given the expression f(x₁,x₂)=sin(x₁) + x₁x₂, the computational graph is:
 *
 *                f(x₁,x₂)
 *                   |
 *                   w₅     w₅=w₄+w₃
 *                  /  \
 *                 /    \
 *  w₄=sin(w₁)    w₄     w₃    w₃=w₁w₂
 *                |   /  |
 *      w₁=x₁     w₁     w₂    w₂=x₃
 *
 * The operations to compute the derivative:
 *
 * w̄₅ = 1 (seed)
 * w̄₄ = w̄₅(∂w₅/∂w₄) = w̄₅
 * w̄₃ = w̄₅(∂w₅/∂w₃) = w̄₅
 * w̄₂ = w̄₃(∂w₃/∂w₂) = w̄₃w₁
 * w̄₁ = w̄₄(∂w₄/∂w₁) + w̄₃(∂w₃/∂w₁) = w̄₄cos(w₁) + w̄₃w₂
 *
 * https://en.wikipedia.org/wiki/Automatic_differentiation#Beyond_forward_and_reverse_accumulation
 */

/**
 * Returns the given variable's gradient tree.
 *
 * @param variable Variable of which to compute the gradient.
 * @param wrt Variables with respect to which to compute the gradient.
 */
VectorXvar GenerateGradientTree(Variable& var, VectorXvar& wrt) {
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetExpression().adjointExpr = MakeShared<Expression>(0.0);
  }

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, SharedPtr<Expression>>> stack;
  stack.reserve(1024);

  stack.emplace_back(var, MakeShared<Expression>(1.0));
  while (!stack.empty()) {
    auto [var, adjoint] = stack.back();
    stack.pop_back();

    auto& varExpr = var.GetExpression();
    auto& lhs = varExpr.args[0];
    auto& rhs = varExpr.args[1];

    if (varExpr.adjointExpr == nullptr) {
      varExpr.adjointExpr = adjoint;
    } else {
      varExpr.adjointExpr = varExpr.adjointExpr + adjoint;
    }

    if (lhs != nullptr) {
      stack.emplace_back(lhs, adjoint * varExpr.gradientFuncs[0](lhs, rhs));

      if (rhs != nullptr) {
        stack.emplace_back(rhs, adjoint * varExpr.gradientFuncs[1](lhs, rhs));
      }
    }
  }

  VectorXvar grad{wrt.rows()};
  for (int row = 0; row < wrt.rows(); ++row) {
    grad(row) = Variable{wrt(row).GetExpression().adjointExpr};
  }

  return grad;
}

}  // namespace

Variable::Variable(double value) : expr{MakeShared<Expression>(value)} {}

Variable::Variable(int value) : expr{MakeShared<Expression>(value)} {}

Variable::Variable(SharedPtr<Expression> expr) : expr{std::move(expr)} {}

Variable& Variable::operator=(double value) {
  if (expr == nullptr) {
    expr = MakeShared<Expression>(value);
  } else {
    GetExpression().value = value;
  }
  return *this;
}

Variable& Variable::operator=(int value) {
  if (expr == nullptr) {
    expr = MakeShared<Expression>(value);
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

double Gradient(Variable var, Variable& wrt) {
  wrt.GetExpression().adjoint = 0.0;

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, double>> stack;
  stack.reserve(1024);

  stack.emplace_back(var, 1.0);
  while (!stack.empty()) {
    auto [var, adjoint] = stack.back();
    stack.pop_back();

    auto& varExpr = var.GetExpression();
    auto& lhs = varExpr.args[0];
    auto& rhs = varExpr.args[1];

    varExpr.adjoint += adjoint;

    if (lhs != nullptr) {
      if (rhs == nullptr) {
        stack.emplace_back(
            lhs, adjoint * varExpr.gradientValueFuncs[0](lhs->value, 0.0));
      } else {
        stack.emplace_back(lhs, adjoint * varExpr.gradientValueFuncs[0](
                                              lhs->value, rhs->value));
        stack.emplace_back(rhs, adjoint * varExpr.gradientValueFuncs[1](
                                              lhs->value, rhs->value));
      }
    }
  }

  return wrt.GetExpression().adjoint;
}

Eigen::VectorXd Gradient(Variable var, VectorXvar& wrt) {
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetExpression().adjoint = 0.0;
  }

  // Stack element contains variable and its adjoint
  std::vector<std::tuple<Variable, double>> stack;
  stack.reserve(1024);

  stack.emplace_back(var, 1.0);
  while (!stack.empty()) {
    auto [var, adjoint] = stack.back();
    stack.pop_back();

    auto& varExpr = var.GetExpression();
    auto& lhs = varExpr.args[0];
    auto& rhs = varExpr.args[1];

    varExpr.adjoint += adjoint;

    if (lhs != nullptr) {
      if (rhs == nullptr) {
        stack.emplace_back(
            lhs, adjoint * varExpr.gradientValueFuncs[0](lhs->value, 0.0));
      } else {
        stack.emplace_back(lhs, adjoint * varExpr.gradientValueFuncs[0](
                                              lhs->value, rhs->value));
        stack.emplace_back(rhs, adjoint * varExpr.gradientValueFuncs[1](
                                              lhs->value, rhs->value));
      }
    }
  }

  Eigen::VectorXd grad{wrt.rows()};
  for (int row = 0; row < wrt.rows(); ++row) {
    grad(row) = wrt(row).GetExpression().adjoint;
  }

  return grad;
}

Eigen::SparseMatrix<double> Jacobian(VectorXvar& variables, VectorXvar& wrt) {
  Eigen::SparseMatrix<double> J{variables.rows(), wrt.rows()};

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < variables.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(variables(row), wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        triplets.emplace_back(row, col, g(col));
      }
    }
  }
  J.setFromTriplets(triplets.begin(), triplets.end());

  return J;
}

Eigen::SparseMatrix<double> Hessian(Variable& variable, VectorXvar& wrt) {
  VectorXvar gradientTree = GenerateGradientTree(variable, wrt);

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < gradientTree.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(gradientTree(row), wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        triplets.emplace_back(row, col, g(col));
      }
    }
  }

  Eigen::SparseMatrix<double> H{wrt.rows(), wrt.rows()};
  H.setFromTriplets(triplets.begin(), triplets.end());

  return H;
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
