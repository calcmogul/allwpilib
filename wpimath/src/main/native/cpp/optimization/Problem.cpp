// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/optimization/Problem.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <fmt/core.h>

#include "Eigen/IterativeLinearSolvers"
#include "Eigen/SparseCore"
#include "frc/autodiff/Gradient.h"
#include "frc/autodiff/Hessian.h"
#include "frc/autodiff/Jacobian.h"
#include "frc/autodiff/Variable.h"
#include "units/time.h"

using namespace frc;

template <typename... Args>
constexpr double Max(double a, double b, Args... args) {
  if constexpr (sizeof...(Args) > 0) {
    return Max(std::max(a, b), args...);
  } else {
    return std::max(a, b);
  }
}

Problem::Problem(ProblemType problemType) : m_problemType{problemType} {
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
  m_config = config;

  if (!m_f.has_value() && m_equalityConstraints.size() == 0 &&
      m_inequalityConstraints.size() == 0) {
    // If there's no cost function or constraints, do nothing
    return SolverStatus::kOk;
  } else if (!m_f.has_value()) {
    // If there's no cost function, make it zero and continue
    m_f = 0.0;
  }

  // Create the initial value column vector
  Eigen::VectorXd x{m_decisionVariables.size(), 1};
  for (size_t i = 0; i < m_decisionVariables.size(); ++i) {
    x(i) = m_decisionVariables[i].Value();
  }

  // Solve the optimization problem
  Eigen::VectorXd solution;
  SolverStatus status = SolverStatus::kOk;
  if (m_problemType == ProblemType::kLinear) {
    // TODO: Precompute gradients once
    solution = InteriorPoint(x, &status);
  } else if (m_problemType == ProblemType::kQuadratic) {
    // TODO: Precompute gradients and hessian once
    solution = InteriorPoint(x, &status);
  } else if (m_problemType == ProblemType::kNonlinear) {
    solution = InteriorPoint(x, &status);
  }

  // Assign solution to the original Variable instances
  SetAD(m_decisionVariables, solution);

  return status;
}

void Problem::SetAD(std::vector<autodiff::Variable>& dest,
                    const Eigen::Ref<const Eigen::VectorXd>& src) {
  assert(dest.size() == static_cast<size_t>(src.rows()));

  for (size_t row = 0; row < dest.size(); ++row) {
    dest[row] = src(row);
  }
}

void Problem::SetAD(Eigen::Ref<autodiff::VectorXvar> dest,
                    const Eigen::Ref<const Eigen::VectorXd>& src) {
  assert(dest.rows() == src.rows());

  for (int row = 0; row < dest.rows(); ++row) {
    dest(row) = src(row);
  }
}

void Problem::Regularize(Eigen::SparseMatrix<double>& A) {
  // See algorithm 3.4 on page 53 of [1].
  //
  // [1] Nocedal, J. and Wright, S. Numerical Optimization, 2nd. ed., Ch. 19.
  //     Springer, 2006.

  constexpr double beta = 0.1;
  constexpr double delta = 0.01;

  Eigen::SparseMatrix<double> L{A.rows(), A.cols()};
  Eigen::SparseMatrix<double> D{A.rows(), A.cols()};

  double c_ij = 0.0;
  double theta_j = 0.0;
  for (int j = 0; j < A.rows(); ++j) {
    double c_jj = A.coeff(j, j);
    for (int s = 0; s < j - 1; ++s) {
      double l_js = L.coeff(j, s);
      c_jj -= D.coeff(s, s) * l_js * l_js;
    }

    theta_j = std::max(theta_j, std::abs(c_ij));
    D.coeffRef(j, j) = std::max(
        std::max(std::abs(c_jj), (theta_j / beta) * (theta_j / beta)), delta);

    for (int i = j + 1; i < A.rows(); ++i) {
      c_ij = A.coeff(i, j);
      for (int s = 0; s < j - 1; ++s) {
        c_ij -= D.coeff(s, s) * L.coeff(i, s) * L.coeff(s, j);
      }
      L.coeffRef(i, j) = c_ij / D.coeff(j, j);
    }
  }

  A = L * D * L.transpose();
}

double Problem::FractionToTheBoundaryRule(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const Eigen::Ref<const Eigen::VectorXd>& p, double tau) {
  // αᵐᵃˣ = max{α ∈ (0, 1] : x + αp ≥ (1−τ)x}
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

double Problem::f(const Eigen::Ref<const Eigen::VectorXd>& x) {
  SetAD(m_decisionVariables, x);
  return m_f.value().Value();
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
  //   αₖᵐᵃˣ = max{α ∈ (0, 1] : xₖ + αpₖˣ ≥ (1−τⱼ)xₖ}
  //   αₖᶻ = max{α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ}
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
    fmt::print("Number of inequality constraints: {}\n",
               m_inequalityConstraints.size());
  }

  // Barrier parameter scale factor κ_μ for tolerance checks
  constexpr double kappa_epsilon = 10.0;

  // Barrier parameter scale factor κ_Σ for inequality constraint Lagrange
  // multiplier safeguard
  constexpr double kappa_sigma = 1e10;

  // Fraction-to-the-boundary rule scale factor minimum
  constexpr double tau_min = 0.995;

  // Tuning parameters for μ update
  constexpr double kappa_mu = 0.2;  // (0, 1)
  constexpr double theta_mu = 1.5;  // (1, 2)

  // Barrier parameter μ
  double mu = 0.1;

  // Fraction-to-the-boundary rule scale factor τ
  double tau = tau_min;

  std::vector<Eigen::Triplet<double>> triplets;

  Eigen::VectorXd x = initialGuess;

  Eigen::VectorXd s = Eigen::VectorXd::Ones(m_inequalityConstraints.size());
  Eigen::VectorXd y = Eigen::VectorXd::Zero(m_equalityConstraints.size(), 1);
  Eigen::VectorXd z = Eigen::VectorXd::Ones(m_inequalityConstraints.size());

  autodiff::VectorXvar sAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());
  autodiff::VectorXvar yAD =
      autodiff::VectorXvar::Zero(m_equalityConstraints.size(), 1);
  autodiff::VectorXvar zAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());

  autodiff::MapVectorXvar decisionVariablesAD(m_decisionVariables.data(),
                                              m_decisionVariables.size());
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

  Eigen::VectorXd step = Eigen::VectorXd::Zero(x.rows(), 1);

  SetAD(m_decisionVariables, x);
  L.Update();

  // Error estimate E_μ
  double E_mu = std::numeric_limits<double>::infinity();

  autodiff::Hessian hessian{L, decisionVariablesAD};

  int iterations = 0;

  auto startTime = std::chrono::system_clock::now();
  auto endTime = startTime;

  while (E_mu > m_config.tolerance) {
    while (E_mu > kappa_epsilon * mu) {
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

      // Hₖ = ∇²ₓₓL(x, s, y, z)ₖ
      Eigen::SparseMatrix<double> H = hessian.Calculate();

      //         [∇ᵀcₑ₁(x)ₖ]
      // Aₑ(x) = [∇ᵀcₑ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcₑₘ(x)ₖ]
      Eigen::SparseMatrix<double> A_e =
          autodiff::Jacobian(c_eAD, decisionVariablesAD);

      //         [∇ᵀcᵢ₁(x)ₖ]
      // Aᵢ(x) = [∇ᵀcᵢ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcᵢₘ(x)ₖ]
      Eigen::SparseMatrix<double> A_i =
          autodiff::Jacobian(c_iAD, decisionVariablesAD);

      // lhs = [H + AᵢᵀΣAᵢ  Aₑᵀ]
      //       [    Aₑ       0 ]
      Eigen::SparseMatrix<double> lhsTop{H.rows(), H.cols() + A_e.rows()};
      lhsTop.leftCols(H.cols()) = H + A_i.transpose() * sigma * A_i;
      lhsTop.rightCols(A_e.rows()) = A_e.transpose();
      Eigen::SparseMatrix<double> lhsBottom{A_e.rows(), H.cols() + A_e.rows()};
      lhsBottom.leftCols(A_e.cols()) = A_e;
      Eigen::SparseMatrix<double, Eigen::RowMajor> lhs{H.rows() + A_e.rows(),
                                                       H.cols() + A_e.rows()};
      lhs.topRows(lhsTop.rows()) = lhsTop;
      lhs.bottomRows(lhsBottom.rows()) = lhsBottom;

      // rhs = −[∇f − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
      //        [               cₑ               ]
      //
      // The outer negative sign is applied in the solve() call.
      Eigen::VectorXd c_e{m_equalityConstraints.size()};
      for (size_t row = 0; row < m_equalityConstraints.size(); row++) {
        c_e[row] = m_equalityConstraints[row].Value();
      }
      Eigen::VectorXd c_i{m_inequalityConstraints.size()};
      for (size_t row = 0; row < m_inequalityConstraints.size(); row++) {
        c_i[row] = m_inequalityConstraints[row].Value();
      }
      Eigen::VectorXd rhs{x.rows() + y.rows()};
      rhs.topRows(x.rows()) =
          autodiff::Gradient(m_f.value(), decisionVariablesAD) -
          A_e.transpose() * y +
          A_i.transpose() * (sigma * c_i - mu * inverseS * e - z);
      rhs.bottomRows(y.rows()) = c_e;

      // Regularize lhs by adding a multiple of the identity matrix
      // TODO: Use LDLT regularization instead? Needs tests.
      // Regularize(lhs);
      Eigen::SparseMatrix<double, Eigen::RowMajor> regularization{lhs.rows(),
                                                                  lhs.cols()};
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

      // αₖᵐᵃˣ = max{α ∈ (0, 1] : sₖ + αpₖˢ ≥ (1−τⱼ)sₖ}
      double alpha_max = FractionToTheBoundaryRule(s, p_s, tau);

      // αₖᶻ = max{α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ}
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
      //   zₖ₊₁⁽ⁱ⁾ = max{min{zₖ₊₁⁽ⁱ⁾, κ_Σ μⱼ/sₖ₊₁⁽ⁱ⁾}, μⱼ/(κ_Σ sₖ₊₁⁽ⁱ⁾)}
      //
      // for some fixed κ_Σ ≥ 1 after each step. See equation (16) in [2].
      for (int row = 0; row < z.rows(); ++row) {
        z(row) = std::max(std::min(z(row), kappa_sigma * mu / s(row)),
                          mu / (kappa_sigma * s(row)));
      }

      SetAD(m_decisionVariables, x);
      SetAD(sAD, s);
      SetAD(yAD, y);
      SetAD(zAD, z);
      L.Update();

      // s_d = max{sₘₐₓ, (||y||₁ + ||z||₁) / (m + n)} / sₘₐₓ
      constexpr double s_max = 100.0;
      double s_d = std::max(s_max, (y.lpNorm<1>() + z.lpNorm<1>()) /
                                       (m_equalityConstraints.size() +
                                        m_inequalityConstraints.size())) /
                   s_max;

      // s_c = max{sₘₐₓ, ||z||₁ / n} / sₘₐₓ
      double s_c =
          std::max(s_max, z.lpNorm<1>() / m_inequalityConstraints.size()) /
          s_max;

      // Update variables needed in error estimate
      A_e = autodiff::Jacobian(c_eAD, decisionVariablesAD);
      A_i = autodiff::Jacobian(c_iAD, decisionVariablesAD);
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
      //   Aᵢᵀcᵢ⁻ → 0
      //   ||(cₑ, cᵢ⁻)|| > ε
      //
      // where cᵢ⁻ = max(0, −cᵢ).
      //
      // See "Infeasibility detection" in section 6 of [3].
      if (m_equalityConstraints.size() > 0 &&
          (A_e.transpose() * c_e).norm() < 1e-2 && c_e.norm() > 1e-2) {
        *status = SolverStatus::kInfeasible;
        return x;
      }
      if (m_inequalityConstraints.size() > 0) {
        Eigen::VectorXd c_i_minus = (-c_i).cwiseMax(0.0);
        if ((A_i.transpose() * c_i_minus).norm() < 1e-2 &&
            c_i_minus.norm() > 1e-2) {
          *status = SolverStatus::kInfeasible;
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
      E_mu = Max((autodiff::Gradient(m_f.value(), decisionVariablesAD) -
                  A_e.transpose() * y - A_i.transpose() * z)
                         .lpNorm<Eigen::Infinity>() /
                     s_d,
                 (S * z - mu * e).lpNorm<Eigen::Infinity>() / s_c,
                 c_e.lpNorm<Eigen::Infinity>(),
                 (c_i - s).lpNorm<Eigen::Infinity>());

      ++iterations;
      if (iterations >= m_config.maxIterations) {
        *status = SolverStatus::kMaxIterations;
        return x;
      }

      endTime = std::chrono::system_clock::now();
      if (units::second_t{endTime - startTime} > m_config.timeout) {
        *status = SolverStatus::kTimeout;
        return x;
      }
    }

    // Update the barrier parameter.
    //
    //   μⱼ₊₁ = max{εₜₒₗ/10, min{κ_μ μⱼ, μⱼ^θ_μ}}
    //
    // See equation (7) in [2].
    mu = std::max(m_config.tolerance / 10.0,
                  std::min(kappa_mu * mu, std::pow(mu, theta_mu)));

    // Update the fraction-to-the-boundary rule scaling factor.
    //
    //   τⱼ = max{τₘᵢₙ, 1 − μⱼ}
    //
    // See equation (8) in [2].
    tau = std::max(tau_min, 1.0 - mu);
  }

  if (m_config.diagnostics) {
    endTime = std::chrono::system_clock::now();
    fmt::print("Number of iterations: {}\n", iterations);

    using std::chrono::duration_cast;
    using std::chrono::microseconds;
    fmt::print(
        "Solve time: {} ms\n",
        duration_cast<microseconds>(endTime - startTime).count() / 1000.0);
  }

  *status = SolverStatus::kOk;
  return x;
}
