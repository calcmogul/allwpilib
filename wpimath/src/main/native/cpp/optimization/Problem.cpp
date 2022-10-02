// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/optimization/Problem.h"

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <fmt/core.h>
#include <wpi/scope>

#include "Eigen/IterativeLinearSolvers"
#include "Eigen/SparseCore"
#include "frc/autodiff/Expression.h"
#include "frc/autodiff/Gradient.h"
#include "frc/autodiff/Hessian.h"
#include "frc/autodiff/Jacobian.h"
#include "frc/autodiff/Variable.h"
#include "units/time.h"

using namespace frc;

namespace {
/**
 * Assigns the contents of a double vector to an autodiff vector.
 *
 * @param dest The autodiff vector.
 * @param src The double vector.
 */
void SetAD(std::vector<autodiff::Variable>& dest,
           const Eigen::Ref<const Eigen::VectorXd>& src) {
  assert(dest.size() == static_cast<size_t>(src.rows()));

  for (size_t row = 0; row < dest.size(); ++row) {
    dest[row] = src(row);
  }
}

/**
 * Assigns the contents of a double vector to an autodiff vector.
 *
 * @param dest The autodiff vector.
 * @param src The double vector.
 */
void SetAD(Eigen::Ref<autodiff::VectorXvar> dest,
           const Eigen::Ref<const Eigen::VectorXd>& src) {
  assert(dest.rows() == src.rows());

  for (int row = 0; row < dest.rows(); ++row) {
    dest(row) = src(row);
  }
}

/**
 * Applies fraction-to-the-boundary rule to a variable and its iterate, then
 * returns a fraction of the iterate step size within (0, 1].
 *
 * @param x The variable.
 * @param p The iterate on the variable.
 * @param tau Fraction-to-the-boundary rule scaling factor.
 * @return Fraction of the iterate step size within (0, 1].
 */
double FractionToTheBoundaryRule(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 const Eigen::Ref<const Eigen::VectorXd>& p,
                                 double tau) {
  // αᵐᵃˣ = max(α ∈ (0, 1] : x + αp ≥ (1−τ)x)
  double alpha = 1;
  for (int i = 0; i < x.rows(); ++i) {
    if (p(i) != 0.0) {
      while (alpha * p(i) < -tau * x(i)) {
        alpha *= 0.999;
      }
    }
  }

  return alpha;
}

/**
 * Adds a sparse matrix to the list of triplets with the given row and column
 * offset.
 *
 * @param[out] triplets The triplet storage.
 * @param[in] rowOffset The row offset for each triplet.
 * @param[in] colOffset The column offset for each triplet.
 * @param[in] mat The matrix to iterate over.
 * @param[in] transpose Whether to transpose mat.
 */
void AssignSparseBlock(std::vector<Eigen::Triplet<double>>& triplets,
                       int rowOffset, int colOffset,
                       const Eigen::SparseMatrix<double>& mat,
                       bool transpose = false) {
  for (int k = 0; k < mat.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it{mat, k}; it; ++it) {
      if (transpose) {
        triplets.emplace_back(rowOffset + it.col(), colOffset + it.row(),
                              it.value());
      } else {
        triplets.emplace_back(rowOffset + it.row(), colOffset + it.col(),
                              it.value());
      }
    }
  }
}

/**
 * Converts std::chrono::duration to a number of milliseconds rounded to three
 * decimals.
 */
template <typename Rep, typename Period = std::ratio<1>>
double ToMilliseconds(const std::chrono::duration<Rep, Period>& duration) {
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  return duration_cast<microseconds>(duration).count() / 1000.0;
}
}  // namespace

Problem::Problem() noexcept {
  m_decisionVariables.reserve(1024);
  m_equalityConstraints.reserve(1024);
  m_inequalityConstraints.reserve(1024);
}

VariableMatrix Problem::DecisionVariable(int rows, int cols) {
  VariableMatrix vars{rows, cols};
  int oldSize = m_decisionVariables.size();

  m_decisionVariables.reserve(m_decisionVariables.size() + rows * cols);

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      m_decisionVariables.emplace_back(0.0);
      vars.Autodiff(row, col) = m_decisionVariables[oldSize + row * cols + col];
    }
  }

  return vars;
}

void Problem::Minimize(const VariableMatrix& cost) {
  assert(cost.Rows() == 1 && cost.Cols() == 1);
  m_f = cost.Autodiff(0, 0);
}

void Problem::Minimize(VariableMatrix&& cost) {
  assert(cost.Rows() == 1 && cost.Cols() == 1);
  m_f = std::move(cost.Autodiff(0, 0));
}

void Problem::Maximize(const VariableMatrix& objective) {
  assert(objective.Rows() == 1 && objective.Cols() == 1);

  // Maximizing an objective function is the same as minimizing its negative
  m_f = -objective.Autodiff(0, 0);
}

void Problem::Maximize(VariableMatrix&& objective) {
  assert(objective.Rows() == 1 && objective.Cols() == 1);

  // Maximizing an objective function is the same as minimizing its negative
  m_f = -std::move(objective.Autodiff(0, 0));
}

void Problem::SubjectTo(EqualityConstraints&& constraint) {
  auto& storage = constraint.constraints;

  m_equalityConstraints.reserve(m_equalityConstraints.size() + storage.size());

  for (size_t i = 0; i < storage.size(); ++i) {
    m_equalityConstraints.emplace_back(std::move(storage[i]));
  }
}

void Problem::SubjectTo(InequalityConstraints&& constraint) {
  auto& storage = constraint.constraints;

  m_inequalityConstraints.reserve(m_inequalityConstraints.size() +
                                  storage.size());

  for (size_t i = 0; i < storage.size(); ++i) {
    m_inequalityConstraints.emplace_back(std::move(storage[i]));
  }
}

SolverStatus Problem::Solve(const SolverConfig& config) {
  constexpr std::array<const char*, 5> kExprTypeToName = {
      "empty", "constant", "linear", "quadratic", "nonlinear"};

  m_config = config;

  // Create the initial value column vector
  Eigen::VectorXd x{m_decisionVariables.size(), 1};
  for (size_t i = 0; i < m_decisionVariables.size(); ++i) {
    x(i) = m_decisionVariables[i].Value();
  }

  SolverStatus status;

  // Get f's expression type
  if (m_f.has_value()) {
    status.costFunctionType = m_f.value().Type();
  }

  // Get the highest order equality constraint expression type
  for (const auto& constraint : m_equalityConstraints) {
    auto constraintType = constraint.Type();
    if (status.equalityConstraintType < constraintType) {
      status.equalityConstraintType = constraintType;
    }
  }

  // Get the highest order inequality constraint expression type
  for (const auto& constraint : m_inequalityConstraints) {
    auto constraintType = constraint.Type();
    if (status.inequalityConstraintType < constraintType) {
      status.inequalityConstraintType = constraintType;
    }
  }

  if (m_config.diagnostics) {
    fmt::print("The cost function is {}.\n",
               kExprTypeToName[static_cast<int>(status.costFunctionType)]);
    fmt::print(
        "The equality constraints are {}.\n",
        kExprTypeToName[static_cast<int>(status.equalityConstraintType)]);
    fmt::print(
        "The inequality constraints are {}.\n",
        kExprTypeToName[static_cast<int>(status.inequalityConstraintType)]);
    fmt::print("\n");
  }

  // If the problem is empty or constant, there's nothing to do
  if (status.costFunctionType <= autodiff::ExpressionType::kConstant &&
      status.equalityConstraintType <= autodiff::ExpressionType::kConstant &&
      status.inequalityConstraintType <= autodiff::ExpressionType::kConstant) {
    return status;
  }

  // If there's no cost function, make it zero and continue
  if (!m_f.has_value()) {
    m_f = 0.0;
  }

  // Solve the optimization problem
  Eigen::VectorXd solution = InteriorPoint(x, &status);

  // Assign the solution to the original Variable instances
  SetAD(m_decisionVariables, solution);

  return status;
}

Eigen::VectorXd Problem::InteriorPoint(
    const Eigen::Ref<const Eigen::VectorXd>& initialGuess,
    SolverStatus* status) {
  // Let f(x)ₖ be the cost function, cₑ(x)ₖ be the equality constraints, and
  // cᵢ(x)ₖ be the inequality constraints. The Lagrangian of the optimization
  // problem is
  //
  //   L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ − zₖᵀ(cᵢ(x)ₖ − sₖ)
  //
  // The Hessian of the Lagrangian is
  //
  //   H(x)ₖ = ∇²ₓₓL(x, s, y, z)ₖ
  //
  // The primal-dual barrier term Hessian Σ is defined as
  //
  //   Σ = S⁻¹Z
  //
  // where
  //
  //       [s₁ 0 ⋯ 0 ]
  //   S = [0  ⋱   ⋮ ]
  //       [⋮    ⋱ 0 ]
  //       [0  ⋯ 0 sₘ]
  //
  //       [z₁ 0 ⋯ 0 ]
  //   Z = [0  ⋱   ⋮ ]
  //       [⋮    ⋱ 0 ]
  //       [0  ⋯ 0 zₘ]
  //
  // and where m is the number of inequality constraints.
  //
  // Let f(x) = f(x)ₖ, H = H(x)ₖ, Aₑ = Aₑ(x)ₖ, and Aᵢ = Aᵢ(x)ₖ for clarity. We
  // want to solve the following Newton-KKT system shown in equation (19.12) of
  // [1].
  //
  //   [H    0  Aₑᵀ  Aᵢᵀ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [0    Σ   0   −I ][ pₖˢ] = −[     z − μS⁻¹e     ]
  //   [Aₑ   0   0    0 ][−pₖʸ]    [        cₑ         ]
  //   [Aᵢ  −I   0    0 ][−pₖᶻ]    [      cᵢ − s       ]
  //
  // where e is a column vector of ones with a number of rows equal to the
  // number of inequality constraints.
  //
  // Solve the second row for pₖˢ.
  //
  //   Σpₖˢ + pₖᶻ = μS⁻¹e − z
  //   Σpₖˢ = μS⁻¹e − z − pₖᶻ
  //   pₖˢ = μΣ⁻¹S⁻¹e − Σ⁻¹z − Σ⁻¹pₖᶻ
  //
  // Substitute Σ = S⁻¹Z into the first two terms.
  //
  //   pₖˢ = μ(S⁻¹Z)⁻¹S⁻¹e − (S⁻¹Z)⁻¹z − Σ⁻¹pₖᶻ
  //   pₖˢ = μZ⁻¹SS⁻¹e − Z⁻¹Sz − Σ⁻¹pₖᶻ
  //   pₖˢ = μZ⁻¹e − s − Σ⁻¹pₖᶻ
  //
  // Substitute the explicit formula for pₖˢ into the fourth row and simplify.
  //
  //   Aᵢpₖˣ − pₖˢ = s − cᵢ
  //   Aᵢpₖˣ − (μZ⁻¹e − s − Σ⁻¹pₖᶻ) = s − cᵢ
  //   Aᵢpₖˣ − μZ⁻¹e + s + Σ⁻¹pₖᶻ = s − cᵢ
  //   Aᵢpₖˣ + Σ⁻¹pₖᶻ = −cᵢ + μZ⁻¹e
  //
  // Substitute the new second and fourth rows into the system.
  //
  //   [H   0  Aₑᵀ  Aᵢᵀ ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [0   I   0    0  ][ pₖˢ] = −[−μZ⁻¹e + s + Σ⁻¹pₖᶻ]
  //   [Aₑ  0   0    0  ][−pₖʸ]    [        cₑ         ]
  //   [Aᵢ  0   0   −Σ⁻¹][−pₖᶻ]    [     cᵢ − μZ⁻¹e    ]
  //
  // Eliminate the second row and column.
  //
  //   [H   Aₑᵀ   Aᵢ ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [Aₑ   0    0  ][−pₖʸ] = −[        cₑ         ]
  //   [Aᵢ   0   −Σ⁻¹][−pₖᶻ]    [    cᵢ − μZ⁻¹e     ]
  //
  // Solve the third row for pₖᶻ.
  //
  //   Aₑpₖˣ + Σ⁻¹pₖᶻ = −cᵢ + μZ⁻¹e
  //   pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ
  //
  // Substitute the explicit formula for pₖᶻ into the first row.
  //
  //   Hpₖˣ − Aₑᵀpₖʸ − Aᵢᵀpₖᶻ = −∇f(x) + Aₑᵀy + Aᵢᵀz
  //   Hpₖˣ − Aₑᵀpₖʸ − Aᵢᵀ(−Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ) = −∇f(x) + Aₑᵀy + Aᵢᵀz
  //
  // Expand and simplify.
  //
  //   Hpₖˣ − Aₑᵀpₖʸ + AᵢᵀΣcᵢ − AᵢᵀμS⁻¹e + AᵢᵀΣAᵢpₖˣ = −∇f(x) + Aₑᵀy + Aᵢᵀz
  //   Hpₖˣ + AᵢᵀΣAᵢpₖˣ − Aₑᵀpₖʸ  = −∇f(x) + Aₑᵀy + AᵢᵀΣcᵢ + AᵢᵀμS⁻¹e + Aᵢᵀz
  //   (H + AᵢᵀΣAᵢ)pₖˣ − Aₑᵀpₖʸ = −∇f(x) + Aₑᵀy − Aᵢᵀ(Σcᵢ − μS⁻¹e − z)
  //   (H + AᵢᵀΣAᵢ)pₖˣ − Aₑᵀpₖʸ = −(∇f(x) − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z))
  //
  // Substitute the new first and third rows into the system.
  //
  //   [H + AᵢᵀΣAᵢ   Aₑᵀ  0][ pₖˣ]    [∇f(x) − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
  //   [    Aₑ        0   0][−pₖʸ] = −[                cₑ                 ]
  //   [    0         0   I][−pₖᶻ]    [       −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ       ]
  //
  // Eliminate the third row and column.
  //
  //   [H + AᵢᵀΣAᵢ  Aₑᵀ][ pₖˣ] = −[∇f(x) − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
  //   [    Aₑ       0 ][−pₖʸ]    [                cₑ                 ]
  //
  // This reduced 2x2 block system gives the iterates pₖˣ and pₖʸ with the
  // iterates pₖᶻ and pₖˢ given by
  //
  //   pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ
  //   pₖˢ = μZ⁻¹e − s − Σ⁻¹pₖᶻ
  //
  // The iterates are applied like so
  //
  //   xₖ₊₁ = xₖ + αₖᵐᵃˣpₖˣ
  //   sₖ₊₁ = xₖ + αₖᵐᵃˣpₖˢ
  //   yₖ₊₁ = xₖ + αₖᶻpₖʸ
  //   zₖ₊₁ = xₖ + αₖᶻpₖᶻ
  //
  // where αₖᵐᵃˣ and αₖᶻ are computed via the fraction-to-the-boundary rule
  // shown in equations (15a) and (15b) of [2].
  //
  //   αₖᵐᵃˣ = max(α ∈ (0, 1] : xₖ + αpₖˣ ≥ (1−τⱼ)xₖ)
  //   αₖᶻ = max(α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ)
  //
  // [1] Nocedal, J. and Wright, S. "Numerical Optimization", 2nd. ed., Ch. 19.
  //     Springer, 2006.
  // [2] Wächter, A. and Biegler, L. "On the implementation of an interior-point
  //     filter line-search algorithm for large-scale nonlinear programming",
  //     2005. http://cepac.cheme.cmu.edu/pasilectures/biegler/ipopt.pdf
  // [3] Byrd, R. and Nocedal J. and Waltz R. "KNITRO: An Integrated Package for
  //     Nonlinear Optimization", 2005.
  //     https://users.iems.northwestern.edu/~nocedal/PDFfiles/integrated.pdf

  if (m_config.diagnostics) {
    fmt::print("Number of equality constraints: {}\n",
               m_equalityConstraints.size());
    fmt::print("Number of inequality constraints: {}\n\n",
               m_inequalityConstraints.size());

    fmt::print("Error tolerance: {}\n\n", m_config.tolerance);
  }

  // Barrier parameter scale factor κ_μ for tolerance checks
  constexpr double kappa_epsilon = 10.0;

  // Barrier parameter scale factor κ_Σ for inequality constraint Lagrange
  // multiplier safeguard
  constexpr double kappa_sigma = 1e10;

  // Fraction-to-the-boundary rule scale factor minimum
  constexpr double tau_min = 0.99;

  // Tuning parameters for μ update
  constexpr double kappa_mu = 0.2;  // (0, 1)
  constexpr double theta_mu = 1.5;  // (1, 2)

  // Barrier parameter μ
  double mu = 0.1;

  // Fraction-to-the-boundary rule scale factor τ
  double tau = tau_min;

  std::vector<Eigen::Triplet<double>> triplets;

  Eigen::VectorXd x = initialGuess;
  autodiff::MapVectorXvar xAD(m_decisionVariables.data(),
                              m_decisionVariables.size());

  Eigen::VectorXd s = Eigen::VectorXd::Ones(m_inequalityConstraints.size());
  autodiff::VectorXvar sAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());

  Eigen::VectorXd y = Eigen::VectorXd::Zero(m_equalityConstraints.size());
  autodiff::VectorXvar yAD =
      autodiff::VectorXvar::Zero(m_equalityConstraints.size());

  Eigen::VectorXd z = Eigen::VectorXd::Ones(m_inequalityConstraints.size());
  autodiff::VectorXvar zAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());

  autodiff::MapVectorXvar c_eAD(m_equalityConstraints.data(),
                                m_equalityConstraints.size());
  autodiff::MapVectorXvar c_iAD(m_inequalityConstraints.data(),
                                m_inequalityConstraints.size());

  const Eigen::MatrixXd e = Eigen::VectorXd::Ones(s.rows());

  // L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ − zₖᵀ(cᵢ(x)ₖ − sₖ)
  autodiff::Variable L = m_f.value();
  if (m_equalityConstraints.size() > 0) {
    L -= yAD.transpose() * c_eAD;
  }
  if (m_inequalityConstraints.size() > 0) {
    L -= zAD.transpose() * (c_iAD - sAD);
  }

  Eigen::VectorXd step = Eigen::VectorXd::Zero(x.rows());

  SetAD(xAD, x);
  L.Update();

  // Error estimate E_μ
  double E_mu = std::numeric_limits<double>::infinity();

  // Gradient of f ∇f
  Eigen::SparseVector<double> gradientF{xAD.rows()};
  if (status->costFunctionType == autodiff::ExpressionType::kConstant) {
    // If the cost function is constant, the gradient is zero.
    gradientF.setZero();
  } else if (status->costFunctionType == autodiff::ExpressionType::kLinear) {
    // If the cost function is linear, initialize the gradient once here since
    // it's constant. Otherwise, initialization is delayed until the loop below.
    gradientF = autodiff::Gradient(m_f.value(), xAD);
  }

  autodiff::Hessian hessian{L, xAD};

  // Hessian of the Lagrangian H
  //
  // Hₖ = ∇²ₓₓL(x, s, y, z)ₖ
  Eigen::SparseMatrix<double> H{xAD.rows(), xAD.rows()};
  if (status->costFunctionType < autodiff::ExpressionType::kQuadratic &&
      status->equalityConstraintType < autodiff::ExpressionType::kQuadratic &&
      status->inequalityConstraintType < autodiff::ExpressionType::kQuadratic) {
    // If the cost function and constraints are less than quadratic, the Hessian
    // is zero.
    H.setZero();
  } else if (status->costFunctionType <= autodiff::ExpressionType::kQuadratic &&
             status->equalityConstraintType <=
                 autodiff::ExpressionType::kQuadratic &&
             status->inequalityConstraintType <=
                 autodiff::ExpressionType::kQuadratic) {
    // The cost function or constraints are at least quadratic. If none are
    // above quadratic, initialize the Hessian once here since it's constant.
    // Otherwise, initialization is delayed until the loop below.
    H = hessian.Calculate();
  }

  // Equality constraints cₑ and equality constraint Jacobian Aₑ
  //
  //         [∇ᵀcₑ₁(x)ₖ]
  // Aₑ(x) = [∇ᵀcₑ₂(x)ₖ]
  //         [    ⋮    ]
  //         [∇ᵀcₑₘ(x)ₖ]
  Eigen::SparseMatrix<double> A_e(m_equalityConstraints.size(), xAD.rows());
  if (status->equalityConstraintType < autodiff::ExpressionType::kLinear) {
    // If the equality constraints are less than linear, the constraint Jacobian
    // is zero.
    A_e.setZero();
  } else if (status->equalityConstraintType ==
             autodiff::ExpressionType::kLinear) {
    // If the equality constraints are linear, initialize Aₑ once here since
    // it's constant. Otherwise, initialization is delayed until the loop below.
    A_e = autodiff::Jacobian(c_eAD, xAD);
  }

  // Inequality constraints cᵢ and inequality constraint Jacobian Aᵢ
  //
  //         [∇ᵀcᵢ₁(x)ₖ]
  // Aᵢ(x) = [∇ᵀcᵢ₂(x)ₖ]
  //         [    ⋮    ]
  //         [∇ᵀcᵢₘ(x)ₖ]
  Eigen::SparseMatrix<double> A_i(m_inequalityConstraints.size(), xAD.rows());
  if (status->inequalityConstraintType < autodiff::ExpressionType::kLinear) {
    // If the inequality constraints are less than linear, the constraint
    // Jacobian is zero.
    A_i.setZero();
  } else if (status->inequalityConstraintType ==
             autodiff::ExpressionType::kLinear) {
    // If the inequality constraints are linear, initialize Aᵢ once here since
    // it's constant. Otherwise, initialization is delayed until the loop below.
    A_i = autodiff::Jacobian(c_iAD, xAD);
  }

  // Equality constraints cₑ
  Eigen::VectorXd c_e{m_equalityConstraints.size()};

  // Inequality constraints cᵢ
  Eigen::VectorXd c_i{m_inequalityConstraints.size()};

  int iterations = 0;

  auto outerStartTime = std::chrono::system_clock::now();

  wpi::scope_exit exit{[&] {
    if (m_config.diagnostics) {
      auto outerEndTime = std::chrono::system_clock::now();
      fmt::print("\nNumber of iterations: {}\n\n", iterations);

      fmt::print("Solve time: {} ms\n\n",
                 ToMilliseconds(outerEndTime - outerStartTime));

      fmt::print("Exit condition: ");
      if (status->exitCondition == SolverExitCondition::kOk) {
        fmt::print("optimal solution found");
      } else if (status->exitCondition == SolverExitCondition::kInfeasible) {
        fmt::print("problem is infeasible");
      } else if (status->exitCondition == SolverExitCondition::kMaxIterations) {
        fmt::print("maximum iterations exceeded");
      } else if (status->exitCondition == SolverExitCondition::kTimeout) {
        fmt::print("solution returned after timeout");
      }
      fmt::print("\n");
    }
  }};

  while (E_mu > m_config.tolerance) {
    while (E_mu > kappa_epsilon * mu) {
      auto innerStartTime = std::chrono::system_clock::now();

      //     [s₁ 0 ⋯ 0 ]
      // S = [0  ⋱   ⋮ ]
      //     [⋮    ⋱ 0 ]
      //     [0  ⋯ 0 sₘ]
      triplets.clear();
      for (int k = 0; k < s.rows(); k++) {
        triplets.emplace_back(k, k, s[k]);
      }
      Eigen::SparseMatrix<double> S{s.rows(), s.rows()};
      S.setFromTriplets(triplets.begin(), triplets.end());

      //     [z₁ 0 ⋯ 0 ]
      // Z = [0  ⋱   ⋮ ]
      //     [⋮    ⋱ 0 ]
      //     [0  ⋯ 0 zₘ]
      triplets.clear();
      for (int k = 0; k < s.rows(); k++) {
        triplets.emplace_back(k, k, z[k]);
      }
      Eigen::SparseMatrix<double> Z{z.rows(), z.rows()};
      Z.setFromTriplets(triplets.begin(), triplets.end());

      // S⁻¹
      Eigen::SparseMatrix<double> inverseS = S.cwiseInverse();

      // Z⁻¹
      Eigen::SparseMatrix<double> inverseZ = Z.cwiseInverse();

      // Σ = S⁻¹Z
      Eigen::SparseMatrix<double> sigma = inverseS * Z;

      // Σ⁻¹ = SZ⁻¹
      Eigen::SparseMatrix<double> inverseSigma = S * inverseZ;

      if (status->costFunctionType > autodiff::ExpressionType::kQuadratic ||
          status->equalityConstraintType > autodiff::ExpressionType::kLinear ||
          status->inequalityConstraintType >
              autodiff::ExpressionType::kLinear) {
        // Hₖ = ∇²ₓₓL(x, s, y, z)ₖ
        H = hessian.Calculate();
      }

      if (status->equalityConstraintType > autodiff::ExpressionType::kLinear) {
        //         [∇ᵀcₑ₁(x)ₖ]
        // Aₑ(x) = [∇ᵀcₑ₂(x)ₖ]
        //         [    ⋮    ]
        //         [∇ᵀcₑₘ(x)ₖ]
        A_e = autodiff::Jacobian(c_eAD, xAD);
      }

      if (status->inequalityConstraintType >
          autodiff::ExpressionType::kLinear) {
        //         [∇ᵀcᵢ₁(x)ₖ]
        // Aᵢ(x) = [∇ᵀcᵢ₂(x)ₖ]
        //         [    ⋮    ]
        //         [∇ᵀcᵢₘ(x)ₖ]
        A_i = autodiff::Jacobian(c_iAD, xAD);
      }

      // Update cₑ and cᵢ
      for (size_t row = 0; row < m_equalityConstraints.size(); row++) {
        c_e[row] = m_equalityConstraints[row].Value();
      }
      for (size_t row = 0; row < m_inequalityConstraints.size(); row++) {
        c_i[row] = m_inequalityConstraints[row].Value();
      }

      // lhs = [H + AᵢᵀΣAᵢ  Aₑᵀ]
      //       [    Aₑ       0 ]
      triplets.clear();
      Eigen::SparseMatrix<double> tmp = H;
      if (m_inequalityConstraints.size() > 0) {
        tmp += A_i.transpose() * sigma * A_i;
      }
      // Assign top-left quadrant
      AssignSparseBlock(triplets, 0, 0, tmp);
      if (m_equalityConstraints.size() > 0) {
        // Assign bottom-left quadrant
        AssignSparseBlock(triplets, tmp.rows(), 0, A_e);

        // Assign top-right quadrant
        AssignSparseBlock(triplets, 0, tmp.rows(), A_e, true);
      }
      Eigen::SparseMatrix<double> lhs{H.rows() + A_e.rows(),
                                      H.cols() + A_e.rows()};
      lhs.setFromTriplets(triplets.begin(), triplets.end());

      if (status->costFunctionType > autodiff::ExpressionType::kLinear) {
        gradientF = autodiff::Gradient(m_f.value(), xAD);
      }

      // rhs = −[∇f − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
      //        [               cₑ               ]
      //
      // The outer negative sign is applied in the solve() call.
      Eigen::VectorXd rhs{x.rows() + y.rows()};
      rhs.topRows(x.rows()) = gradientF;
      if (m_equalityConstraints.size() > 0) {
        rhs.topRows(x.rows()) -= A_e.transpose() * y;
      }
      if (m_inequalityConstraints.size() > 0) {
        rhs.topRows(x.rows()) +=
            A_i.transpose() * (sigma * c_i - mu * inverseS * e - z);
      }
      rhs.bottomRows(y.rows()) = c_e;

      // Regularize lhs by adding a multiple of the identity matrix
      Eigen::SparseMatrix<double> regularization{lhs.rows(), lhs.cols()};
      regularization.setIdentity();
      regularization *= 1e-9;
      lhs += regularization;

      // Solve the Newton-KKT system
      Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
                               Eigen::Lower | Eigen::Upper>
          solver;
      solver.compute(lhs);
      step = solver.solve(-rhs);

      // step = [ pₖˣ]
      //        [−pₖʸ]
      Eigen::VectorXd p_x = step.segment(0, x.rows());
      Eigen::VectorXd p_y = -step.segment(x.rows(), y.rows());

      // pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ
      Eigen::VectorXd p_z =
          -sigma * c_i + mu * inverseS * e - sigma * A_i * p_x;

      // pₖˢ = μZ⁻¹e − s − Σ⁻¹pₖᶻ
      Eigen::VectorXd p_s = mu * inverseZ * e - s - inverseSigma * p_z;

      // αₖᵐᵃˣ = max(α ∈ (0, 1] : sₖ + αpₖˢ ≥ (1−τⱼ)sₖ)
      double alpha_max = FractionToTheBoundaryRule(s, p_s, tau);

      // αₖᶻ = max(α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ)
      double alpha_z = FractionToTheBoundaryRule(z, p_z, tau);

      // xₖ₊₁ = xₖ + αₖᵐᵃˣpₖˣ
      // sₖ₊₁ = xₖ + αₖᵐᵃˣpₖˢ
      // yₖ₊₁ = xₖ + αₖᶻpₖʸ
      // zₖ₊₁ = xₖ + αₖᶻpₖᶻ
      x += alpha_max * p_x;
      s += alpha_max * p_s;
      y += alpha_z * p_y;
      z += alpha_z * p_z;

      // A requirement for the convergence proof is that the "primal-dual
      // barrier term Hessian" Σₖ does not deviate arbitrarily much from the
      // "primal Hessian" μⱼSₖ⁻². We ensure this by resetting
      //
      //   zₖ₊₁⁽ⁱ⁾ = max(min(zₖ₊₁⁽ⁱ⁾, κ_Σ μⱼ/sₖ₊₁⁽ⁱ⁾), μⱼ/(κ_Σ sₖ₊₁⁽ⁱ⁾))
      //
      // for some fixed κ_Σ ≥ 1 after each step. See equation (16) in [2].
      for (int row = 0; row < z.rows(); ++row) {
        z(row) = std::max(std::min(z(row), kappa_sigma * mu / s(row)),
                          mu / (kappa_sigma * s(row)));
      }

      SetAD(xAD, x);
      SetAD(sAD, s);
      SetAD(yAD, y);
      SetAD(zAD, z);
      L.Update();

      // s_d = max(sₘₐₓ, (||y||₁ + ||z||₁) / (m + n)) / sₘₐₓ
      constexpr double s_max = 100.0;
      double s_d = std::max(s_max, (y.lpNorm<1>() + z.lpNorm<1>()) /
                                       (m_equalityConstraints.size() +
                                        m_inequalityConstraints.size())) /
                   s_max;

      // s_c = max(sₘₐₓ, ||z||₁ / n) / sₘₐₓ
      double s_c =
          std::max(s_max, z.lpNorm<1>() / m_inequalityConstraints.size()) /
          s_max;

      // Update variables needed in error estimate
      if (status->costFunctionType > autodiff::ExpressionType::kLinear) {
        gradientF = autodiff::Gradient(m_f.value(), xAD);
      }
      if (status->equalityConstraintType > autodiff::ExpressionType::kLinear) {
        A_e = autodiff::Jacobian(c_eAD, xAD);
      }
      if (status->inequalityConstraintType >
          autodiff::ExpressionType::kLinear) {
        A_i = autodiff::Jacobian(c_iAD, xAD);
      }
      for (size_t row = 0; row < m_equalityConstraints.size(); row++) {
        c_e[row] = m_equalityConstraints[row].Value();
      }
      for (size_t row = 0; row < m_inequalityConstraints.size(); row++) {
        c_i[row] = m_inequalityConstraints[row].Value();
      }
      triplets.clear();
      for (int k = 0; k < s.rows(); k++) {
        triplets.emplace_back(k, k, s[k]);
      }
      S.setFromTriplets(triplets.begin(), triplets.end());

      // Check for problem infeasibility. The problem is infeasible if
      //
      //   Aₑᵀcₑ → 0
      //   Aᵢᵀcᵢ⁺ → 0
      //   ||(cₑ, cᵢ⁺)|| > ε
      //
      // where cᵢ⁺ = min(cᵢ, 0).
      //
      // See "Infeasibility detection" in section 6 of [3].
      //
      // cᵢ⁺ is used instead of cᵢ⁻ from the paper to follow the convention that
      // feasible inequality constraints are ≥ 0.
      if (m_equalityConstraints.size() > 0 &&
          (A_e.transpose() * c_e).norm() < 1e-6 && c_e.norm() > 1e-2) {
        if (m_config.diagnostics) {
          fmt::print(
              "The problem is infeasible due to violated equality "
              "constraints.\n");
          fmt::print(
              "Violated constraints (cₑ(x) = 0) in order of declaration:\n");
          for (int row = 0; row < c_e.rows(); ++row) {
            if (c_e(row) < 0.0) {
              fmt::print("  {}/{}: {} = 0\n", row + 1, c_e.rows(), c_e(row));
            }
          }
        }

        status->exitCondition = SolverExitCondition::kInfeasible;
        return x;
      }
      if (m_inequalityConstraints.size() > 0) {
        Eigen::VectorXd c_i_plus = c_i.cwiseMin(0.0);
        if ((A_i.transpose() * c_i_plus).norm() < 1e-6 &&
            c_i_plus.norm() > 1e-6) {
          if (m_config.diagnostics) {
            fmt::print(
                "The problem is infeasible due to violated inequality "
                "constraints.\n");
            fmt::print(
                "Violated constraints (cᵢ(x) ≥ 0) in order of declaration:\n");
            for (int row = 0; row < c_i.rows(); ++row) {
              if (c_i(row) < 0.0) {
                fmt::print("  {}/{}: {} ≥ 0\n", row + 1, c_i.rows(), c_i(row));
              }
            }
          }

          status->exitCondition = SolverExitCondition::kInfeasible;
          return x;
        }
      }

      // Update the error estimate using the KKT conditions from equations
      // (19.5a) through (19.5d) in [1].
      //
      //   ∇f − Aₑᵀy − Aᵢᵀz = 0
      //   Sz − μe = 0
      //   cₑ = 0
      //   cᵢ − s = 0
      //
      // The error tolerance is the max of the following infinity norms scaled
      // by s_d and s_c (see equation (5) in [2]).
      //
      //   ||∇f − Aₑᵀy − Aᵢᵀz||_∞ / s_d
      //   ||Sz − μe||_∞ / s_c
      //   ||cₑ||_∞
      //   ||cᵢ − s||_∞
      Eigen::VectorXd eq1 = gradientF;
      if (m_equalityConstraints.size() > 0) {
        eq1 -= A_e.transpose() * y;
      }
      if (m_inequalityConstraints.size() > 0) {
        eq1 -= A_i.transpose() * z;
      }
      E_mu = std::max(eq1.lpNorm<Eigen::Infinity>() / s_d,
                      (S * z - mu * e).lpNorm<Eigen::Infinity>() / s_c);
      if (m_equalityConstraints.size() > 0) {
        E_mu = std::max(E_mu, c_e.lpNorm<Eigen::Infinity>());
      }
      if (m_inequalityConstraints.size() > 0) {
        E_mu = std::max(E_mu, (c_i - s).lpNorm<Eigen::Infinity>());
      }

      auto innerEndTime = std::chrono::system_clock::now();

      if (m_config.diagnostics) {
        if (iterations % 20 == 0) {
          fmt::print("iter  duration (ms)    error\n");
          fmt::print("==============================\n");
        }
        fmt::print("{:>4}  {:>10}     {:>9.3e}\n", iterations,
                   ToMilliseconds(innerEndTime - innerStartTime), E_mu);
      }

      ++iterations;
      if (iterations >= m_config.maxIterations) {
        status->exitCondition = SolverExitCondition::kMaxIterations;
        return x;
      }

      if (units::second_t{innerEndTime - outerStartTime} > m_config.timeout) {
        status->exitCondition = SolverExitCondition::kTimeout;
        return x;
      }
    }

    // Update the barrier parameter.
    //
    //   μⱼ₊₁ = max(εₜₒₗ/10, min(κ_μ μⱼ, μⱼ^θ_μ))
    //
    // See equation (7) in [2].
    mu = std::max(m_config.tolerance / 10.0,
                  std::min(kappa_mu * mu, std::pow(mu, theta_mu)));

    // Update the fraction-to-the-boundary rule scaling factor.
    //
    //   τⱼ = max(τₘᵢₙ, 1 − μⱼ)
    //
    // See equation (8) in [2].
    tau = std::max(tau_min, 1.0 - mu);
  }

  return x;
}
