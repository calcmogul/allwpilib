// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Variable.h"

#include <cmath>

#include <wpi/SymbolExports.h>

#include "frc/autodiff/Tape.h"

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
 * Generates the given variable's gradient tree.
 *
 * @param variable Variable of which to compute the gradient.
 * @param adjoint Variable adjoint of the given variable.
 */
void GenerateVariableAdjoints(Variable& var, Variable adjoint) {
  auto& varExpr = var.GetExpression();
  auto& lhs = varExpr.args[0];
  auto& rhs = varExpr.args[1];

  varExpr.adjointVar += adjoint;

  if (lhs.index != -1) {
    GenerateVariableAdjoints(lhs, adjoint * varExpr.gradientFuncs[0](lhs, rhs));

    if (rhs.index != -1) {
      GenerateVariableAdjoints(rhs,
                               adjoint * varExpr.gradientFuncs[1](lhs, rhs));
    }
  }
}

/**
 * Returns the given variable's gradient tree.
 *
 * @param variable Variable of which to compute the gradient.
 * @param wrt Variables with respect to which to compute the gradient.
 */
VectorXvar GenerateVariableAdjoints(Variable& var, VectorXvar& wrt) {
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetExpression().adjointVar = Constant(0.0);
  }

  Variable adj = Constant(1.0);
  GenerateVariableAdjoints(var, adj);

  VectorXvar grad{wrt.rows()};
  for (int row = 0; row < wrt.rows(); ++row) {
    grad(row) = wrt(row).GetExpression().adjointVar;
  }

  return grad;
}

/**
 * Accumulates the adjoints for the given variable.
 *
 * @param variable Variable of which to compute the gradient.
 * @param adjoint Variable adjoint of the given variable.
 */
void GenerateDoubleAdjoints(Variable& var, double adjoint) {
  auto& varExpr = var.GetExpression();
  auto& lhs = varExpr.args[0];
  auto& rhs = varExpr.args[1];

  varExpr.adjoint += adjoint;

  if (lhs.index != -1) {
    GenerateDoubleAdjoints(
        lhs, adjoint * varExpr.gradientValueFuncs[0](lhs.Value(), rhs.Value()));

    if (rhs.index != -1) {
      GenerateDoubleAdjoints(rhs, adjoint * varExpr.gradientValueFuncs[1](
                                                lhs.Value(), rhs.Value()));
    }
  }
}

}  // namespace

Variable::Variable(double value) {
  *this = Tape::GetTape().PushNullary(value);
}

Variable::Variable(int value) {
  *this = Tape::GetTape().PushNullary(value);
}

Variable::Variable(int index, const PrivateInit&) : index{index} {}

Variable& Variable::operator=(double value) {
  if (index == -1) {
    *this = Tape::GetTape().PushNullary(value);
  } else {
    GetExpression().value = value;
  }
  return *this;
}

Variable& Variable::operator=(int value) {
  if (index == -1) {
    *this = Tape::GetTape().PushNullary(value);
  } else {
    GetExpression().value = value;
  }
  return *this;
}

WPILIB_DLLEXPORT Variable operator*(double lhs, const Variable& rhs) {
  return Constant(lhs) * rhs;
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, double rhs) {
  return lhs * Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, const Variable& rhs) {
  return Tape::GetTape().PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs * rhs; },
      [](double lhs, double rhs) { return rhs; },
      [](const Variable& lhs, const Variable& rhs) { return rhs; },
      [](double lhs, double rhs) { return lhs; },
      [](const Variable& lhs, const Variable& rhs) { return lhs; });
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
  return Constant(lhs) / rhs;
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, double rhs) {
  return lhs / Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, const Variable& rhs) {
  return Tape::GetTape().PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs / rhs; },
      [](double lhs, double rhs) { return 1.0 / rhs; },
      [](const Variable& lhs, const Variable& rhs) { return 1.0 / rhs; },
      [](double lhs, double rhs) { return -lhs / (rhs * rhs); },
      [](const Variable& lhs, const Variable& rhs) {
        return -lhs / (rhs * rhs);
      });
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
  return Constant(lhs) + rhs;
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, double rhs) {
  return lhs + Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, const Variable& rhs) {
  return Tape::GetTape().PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs + rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(1.0); },
      [](double lhs, double rhs) { return 1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(1.0); });
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
  return Constant(lhs) - rhs;
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, double rhs) {
  return lhs - Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, const Variable& rhs) {
  return Tape::GetTape().PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs - rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(1.0); },
      [](double lhs, double rhs) { return -1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(-1.0); });
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
  return Tape::GetTape().PushUnary(
      lhs, [](double lhs, double) { return -lhs; },
      [](double lhs, double rhs) { return -1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(-1.0); });
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs) {
  return Tape::GetTape().PushUnary(
      lhs, [](double lhs, double) { return lhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const Variable& lhs, const Variable& rhs) { return Constant(1.0); });
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
  if (index == -1) {
    return 0.0;
  } else {
    return GetExpression().value;
  }
}

void Variable::Update() {
  GetExpression().Update();
}

const Expression& Variable::GetExpression() const {
  return Tape::GetTape()[index];
}

Expression& Variable::GetExpression() {
  return Tape::GetTape()[index];
}

Variable Constant(double value) {
  return Tape::GetTape().PushNullary(value);
}

double Gradient(Variable var, Variable& wrt) {
  wrt.GetExpression().adjoint = 0.0;
  GenerateDoubleAdjoints(var, 1.0);

  return wrt.GetExpression().adjoint;
}

Eigen::VectorXd Gradient(Variable var, VectorXvar& wrt) {
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetExpression().adjoint = 0.0;
  }
  GenerateDoubleAdjoints(var, 1.0);

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
  int tapeSize = Tape::GetTape().Size();

  VectorXvar gradientTree = GenerateVariableAdjoints(variable, wrt);

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < gradientTree.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(gradientTree(row), wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        triplets.emplace_back(row, col, g(col));
      }
    }
  }

  Tape::GetTape().Resize(tapeSize);

  Eigen::SparseMatrix<double> H{wrt.rows(), wrt.rows()};
  H.setFromTriplets(triplets.begin(), triplets.end());

  return H;
}

Variable abs(double x) {
  return autodiff::abs(Constant(x));
}

Variable abs(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::abs(x); },
      [](double x, double) {
        if (x < 0.0) {
          return -1.0;
        } else if (x > 0.0) {
          return 1.0;
        } else {
          return 0.0;
        }
      },
      [](const Variable& x, const Variable&) {
        if (x.Value() < 0.0) {
          return Constant(-1.0);
        } else if (x.Value() > 0.0) {
          return Constant(1.0);
        } else {
          return Constant(0.0);
        }
      })};
}

Variable acos(double x) {
  return autodiff::acos(Constant(x));
}

Variable acos(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::acos(x); },
      [](double x, double) { return -1.0 / std::sqrt(1.0 - x * x); },
      [](const Variable& x, const Variable&) {
        return -1.0 / autodiff::sqrt(1.0 - x * x);
      })};
}

Variable asin(double x) {
  return autodiff::asin(Constant(x));
}

Variable asin(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::asin(x); },
      [](double x, double) { return 1.0 / std::sqrt(1.0 - x * x); },
      [](const Variable& x, const Variable&) {
        return 1.0 / autodiff::sqrt(1.0 - x * x);
      })};
}

Variable atan(double x) {
  return autodiff::atan(Constant(x));
}

Variable atan(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::atan(x); },
      [](double x, double) { return 1.0 / (1.0 + x * x); },
      [](const Variable& x, const Variable&) { return 1.0 / (1.0 + x * x); })};
}

Variable atan2(double y, const Variable& x) {
  return autodiff::atan2(Constant(y), x);
}

Variable atan2(const Variable& y, double x) {
  return autodiff::atan2(y, Constant(x));
}

Variable atan2(const Variable& y, const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushBinary(
      y, x, [](double y, double x) { return std::atan2(y, x); },
      [](double y, double x) { return x / (y * y + x * x); },
      [](const Variable& y, const Variable& x) { return x / (y * y + x * x); },
      [](double y, double x) { return -y / (y * y + x * x); },
      [](const Variable& y, const Variable& x) {
        return -y / (y * y + x * x);
      })};
}

Variable cos(double x) {
  return autodiff::cos(Constant(x));
}

Variable cos(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::cos(x); },
      [](double x, double) { return -std::sin(x); },
      [](const Variable& x, const Variable&) { return -autodiff::sin(x); })};
}

Variable cosh(double x) {
  return autodiff::cosh(Constant(x));
}

Variable cosh(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::cosh(x); },
      [](double x, double) { return std::sinh(x); },
      [](const Variable& x, const Variable&) { return autodiff::sinh(x); })};
}

Variable erf(double x) {
  return autodiff::erf(Constant(x));
}

Variable erf(const Variable& x) {
  static constexpr double sqrt_pi =
      1.7724538509055160272981674833411451872554456638435L;

  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::erf(x); },
      [](double x, double) { return 2.0 / sqrt_pi * std::exp(-x * x); },
      [](const Variable& x, const Variable&) {
        return 2.0 / sqrt_pi * autodiff::exp(-x * x);
      })};
}

Variable exp(double x) {
  return autodiff::exp(Constant(x));
}

Variable exp(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::exp(x); },
      [](double x, double) { return std::exp(x); },
      [](const Variable& x, const Variable&) { return autodiff::exp(x); })};
}

Variable hypot(double x, const Variable& y) {
  return autodiff::hypot(Constant(x), y);
}

Variable hypot(const Variable& x, double y) {
  return autodiff::hypot(x, Constant(y));
}

Variable hypot(const Variable& x, const Variable& y) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushBinary(
      x, y, [](double x, double y) { return std::hypot(x, y); },
      [](double x, double y) { return x / std::hypot(x, y); },
      [](const Variable& x, const Variable& y) {
        return x / autodiff::hypot(x, y);
      },
      [](double x, double y) { return y / std::hypot(x, y); },
      [](const Variable& x, const Variable& y) {
        return y / autodiff::hypot(x, y);
      })};
}

Variable log(double x) {
  return autodiff::log(Constant(x));
}

Variable log(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::log(x); },
      [](double x, double) { return 1.0 / x; },
      [](const Variable& x, const Variable&) { return 1.0 / x; })};
}

Variable log10(double x) {
  return autodiff::log10(Constant(x));
}

Variable log10(const Variable& x) {
  static constexpr double ln10 = 2.3025850929940456840179914546843L;

  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::log10(x); },
      [](double x, double) { return 1.0 / (ln10 * x); },
      [](const Variable& x, const Variable&) { return 1.0 / (ln10 * x); })};
}

Variable pow(double base, const Variable& power) {
  return autodiff::pow(Constant(base), power);
}

Variable pow(const Variable& base, double power) {
  return autodiff::pow(base, Constant(power));
}

Variable pow(const Variable& base, const Variable& power) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushBinary(
      base, power,
      [](double base, double power) { return std::pow(base, power); },
      [](double base, double power) {
        return std::pow(base, power - 1) * power;
      },
      [](const Variable& base, const Variable& power) {
        return autodiff::pow(base, power - 1) * power;
      },
      [](double base, double power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base == 0.0) {
          return 0.0;
        } else {
          return std::pow(base, power - 1) * base * std::log(base);
        }
      },
      [](const Variable& base, const Variable& power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base.Value() == 0.0) {
          return Constant(0.0);
        } else {
          return autodiff::pow(base, power - 1) * base * autodiff::log(base);
        }
      })};
}

Variable sin(double x) {
  return autodiff::sin(Constant(x));
}

Variable sin(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::sin(x); },
      [](double x, double) { return std::cos(x); },
      [](const Variable& x, const Variable&) { return autodiff::cos(x); })};
}

Variable sinh(double x) {
  return autodiff::sinh(Constant(x));
}

Variable sinh(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::sinh(x); },
      [](double x, double) { return std::cosh(x); },
      [](const Variable& x, const Variable&) { return autodiff::cosh(x); })};
}

Variable sqrt(double x) {
  return autodiff::sqrt(Constant(x));
}

Variable sqrt(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::sqrt(x); },
      [](double x, double) { return 1.0 / (2.0 * std::sqrt(x)); },
      [](const Variable& x, const Variable&) {
        return 1.0 / (2.0 * autodiff::sqrt(x));
      })};
}

Variable tan(double x) {
  return autodiff::tan(Constant(x));
}

Variable tan(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::tan(x); },
      [](double x, double) { return 1.0 / (std::cos(x) * std::cos(x)); },
      [](const Variable& x, const Variable&) {
        return 1.0 / (autodiff::cos(x) * autodiff::cos(x));
      })};
}

Variable tanh(double x) {
  return autodiff::tanh(Constant(x));
}

Variable tanh(const Variable& x) {
  auto& tape = Tape::GetTape();
  return Variable{tape.PushUnary(
      x, [](double x, double) { return std::tanh(x); },
      [](double x, double) { return 1.0 / (std::cosh(x) * std::cosh(x)); },
      [](const Variable& x, const Variable&) {
        return 1.0 / (autodiff::cosh(x) * autodiff::cosh(x));
      })};
}

}  // namespace frc::autodiff
