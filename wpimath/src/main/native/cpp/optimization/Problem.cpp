// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/optimization/Problem.h"

#include <cassert>
#include <cmath>
#include <vector>

#include "Eigen/IterativeLinearSolvers"
#include "Eigen/SparseCore"
#include "frc/autodiff/Variable.h"

using namespace frc;

Problem::Problem(ProblemType problemType) : m_problemType{problemType} {}

VariableMatrix Problem::DecisionVariable(int rows, int cols) {
  VariableMatrix vars{rows, cols};

  int oldSize = m_leaves.rows();
  m_leaves.resize(oldSize + rows * cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      m_leaves[oldSize + row * cols + col] = autodiff::Variable{0.0};
      vars.Autodiff(row, col) = m_leaves[oldSize + row * cols + col];
    }
  }

  return vars;
}

void Problem::Minimize(const autodiff::Variable& cost) {
  m_f = cost;
}

void Problem::Minimize(autodiff::Variable&& cost) {
  m_f = std::move(cost);
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
  int oldSize = m_equalityConstraints.rows();
  auto& storage = constraint.constraints;

  // We move the autodiff variables into a temporary during the resize, then
  // back, because resize() makes all the gradients it contains always return
  // zero after that
  autodiff::VectorXvar newConstraintStorage{
      m_equalityConstraints.rows() + storage.size(), 1};
  for (int row = 0; row < m_equalityConstraints.rows(); ++row) {
    newConstraintStorage(row) = std::move(m_equalityConstraints(row));
  }
  m_equalityConstraints.resize(m_equalityConstraints.rows() + storage.size());
  m_equalityConstraints = std::move(newConstraintStorage);

  for (size_t i = 0; i < storage.size(); ++i) {
    m_equalityConstraints(oldSize + i) = std::move(storage[i]);
  }
}

void Problem::SubjectTo(InequalityConstraints&& constraint) {
  int oldSize = m_inequalityConstraints.rows();
  auto& storage = constraint.constraints;

  // We move the autodiff variables into a temporary during the resize, then
  // back, because resize() makes all the gradients it contains always return
  // zero after that
  autodiff::VectorXvar newConstraintStorage{
      m_inequalityConstraints.rows() + storage.size(), 1};
  for (int row = 0; row < m_inequalityConstraints.rows(); ++row) {
    newConstraintStorage(row) = std::move(m_inequalityConstraints(row));
  }
  m_inequalityConstraints.resize(m_inequalityConstraints.rows() +
                                 storage.size());
  m_inequalityConstraints = std::move(newConstraintStorage);

  for (size_t i = 0; i < storage.size(); ++i) {
    m_inequalityConstraints(oldSize + i) = std::move(storage[i]);
  }
}

void Problem::Solve(double tolerance, int maxIterations) {
  m_tolerance = tolerance;
  m_maxIterations = maxIterations;

  if (!m_f.has_value() && m_equalityConstraints.rows() == 0 &&
      m_inequalityConstraints.rows() == 0) {
    // If there's no cost function or constraints, do nothing
    return;
  } else if (!m_f.has_value()) {
    // If there's no cost function, make it zero and continue
    m_f = 0.0;
  }

  // Create the initial value column vector
  Eigen::VectorXd x{m_leaves.size(), 1};
  for (int i = 0; i < m_leaves.rows(); ++i) {
    x(i) = m_leaves(i).Value();
  }

  // Solve the optimization problem
  Eigen::VectorXd solution;
  if (m_problemType == ProblemType::kLinear) {
    solution = InteriorPoint(x);
  } else if (m_problemType == ProblemType::kQuadratic) {
    // TODO: Use OSQP?
    solution = InteriorPoint(x);
  } else if (m_problemType == ProblemType::kNonlinear) {
    solution = InteriorPoint(x);
  }

  // Assign solution to the original Variable instances
  SetAD(m_leaves, solution);
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
    double c_jj = A.coeffRef(j, j);
    for (int s = 0; s < j - 1; ++s) {
      double l_js = L.coeffRef(j, s);
      c_jj -= D.coeffRef(s, s) * l_js * l_js;
    }

    theta_j = std::max(theta_j, std::abs(c_ij));
    D.coeffRef(j, j) = std::max(
        std::max(std::abs(c_jj), (theta_j / beta) * (theta_j / beta)), delta);

    for (int i = j + 1; i < A.rows(); ++i) {
      c_ij = A.coeffRef(i, j);
      for (int s = 0; s < j - 1; ++s) {
        c_ij -= D.coeffRef(s, s) * L.coeffRef(i, s) * L.coeffRef(s, j);
      }
      L.coeffRef(i, j) = c_ij / D.coeffRef(j, j);
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
    if (p[i] != 0) {
      while (alpha * p[i] < -tau * x[i]) {
        alpha *= 0.999;
      }
    }
  }

  return alpha;
}

double Problem::f(const Eigen::Ref<const Eigen::VectorXd>& x) {
  SetAD(m_leaves, x);
  return m_f.value().Value();
}

Eigen::VectorXd Problem::InteriorPoint(
    const Eigen::Ref<const Eigen::VectorXd>& initialGuess) {
  // Let f(x)ₖ be the cost function, cₑ(x)ₖ be the equality constraints, and
  // cᵢ(x)ₖ be the inequality constraints.
  //
  //   L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ − zₖᵀ(cᵢ(x)ₖ − sₖ)
  //
  //   H(x)ₖ = ∇²ₓₓL(x, s, y, z)ₖ
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
  // where m is the number of inequality constraints.
  //
  //   Σ = S⁻¹Z
  //
  // e is a column vector of ones with a number of rows equal to the number of
  // inequality constraints.
  //
  // Let f(x) = f(x)ₖ, H = H(x)ₖ, Aₑ = Aₑ(x)ₖ, and Aᵢ = Aᵢ(x)ₖ for clarity.
  //
  // We want to solve the Newton-KKT system shown in equation (19.12) in [1].
  //
  //   [H    0  Aₑᵀ  Aᵢᵀ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [0    Σ   0   −I ][ pₖˢ] = −[     z − μS⁻¹e     ]
  //   [Aₑ   0   0    0 ][-pₖʸ]    [        cₑ         ]
  //   [Aᵢ  −I   0    0 ][-pₖᶻ]    [      cᵢ − s       ]
  //
  // Take the second row.
  //
  //   Σpₖˢ + pₖᶻ = μS⁻¹e − z
  //
  // Solve for pₖˢ.
  //
  //   pₖˢ = μΣ⁻¹S⁻¹e − Σ⁻¹z − Σ⁻¹pₖᶻ
  //
  // Substitute Σ = S⁻¹Z into the first two elements.
  //
  //   pₖˢ = μZ⁻¹e − s − Σ⁻¹pₖᶻ
  //
  // Take the fourth row.
  //
  //   Aₑpₖˣ − pₖˢ = s − cᵢ
  //
  // Substitute the explicit formula for pₖˢ.
  //
  //   Aₑpₖˣ − μZ⁻¹e + s + Σ⁻¹pₖᶻ = s − cᵢ
  //
  // Simplify.
  //
  //   Aₑpₖˣ + Σ⁻¹pₖᶻ = −cᵢ + μZ⁻¹e
  //
  // Substitute the new second and fourth rows into the system.
  //
  //   [H   0  Aₑᵀ  Aᵢᵀ ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [0   I   0    0  ][ pₖˢ] = −[     z − μS⁻¹e     ]
  //   [Aₑ  0   0    0  ][−pₖʸ]    [        cₑ         ]
  //   [Aᵢ  0   0   −Σ⁻¹][−pₖᶻ]    [     cᵢ − μZ⁻¹e    ]
  //
  // Eliminate the second row and column.
  //
  //   [H   Aₑᵀ   Aᵢ ][ pₖˣ]    [∇f(x) − Aₑᵀy − Aᵢᵀz]
  //   [Aₑ   0    0  ][−pₖʸ] = −[        cₑ         ]
  //   [Aᵢ   0   −Σ⁻¹][−pₖᶻ]    [    cᵢ − μZ⁻¹e     ]
  //
  // Take the third row.
  //
  //   Aₑpₖˣ + Σ⁻¹pₖᶻ = −cᵢ + μZ⁻¹e
  //
  // Solve for pₖᶻ.
  //
  //   pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ
  //
  // Take the first row.
  //
  //   Hpₖˣ − Aₑᵀpₖʸ − Aᵢᵀpₖᶻ = −∇f(x) + Aₑᵀy + Aᵢᵀz
  //
  // Substitute the explicit formula for pₖᶻ.
  //
  //   Hpₖˣ − Aₑᵀpₖʸ − Aᵢᵀ(−Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ) = −∇f(x) + Aₑᵀy + Aᵢᵀz
  //
  // Expand and simplify.
  //
  //   (H + AᵢᵀΣAᵢ)pₖˣ − Aₑᵀpₖʸ = −∇f(x) + Aₑᵀy − Aᵢᵀ(Σcᵢ − μS⁻¹e − z)
  //
  // Substitute the new first and third rows into the system.
  //
  //   [H   Aₑᵀ  0][ pₖˣ]    [∇f(x) − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
  //   [Aₑ   0   0][−pₖʸ] = −[                cₑ                 ]
  //   [0    0   I][−pₖᶻ]    [       −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ       ]
  //
  // Eliminate the third row and column.
  //
  //   [H + AᵢᵀΣAᵢ  Aₑᵀ][ pₖˣ] = −[∇f(x) − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
  //   [    Aₑ       0 ][−pₖʸ]    [                cₑ                 ]
  //
  // The iterate pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ.
  // The iterate pₖˢ = μZ⁻¹e − Σ⁻¹z − Σ⁻¹pₖᶻ.
  //
  // The fraction-to-the-boundary rule is used to compute αₖᵐᵃˣ and αₖᶻ. See
  // equations (15a) and (15b) in [2].
  //
  //   αₖᵐᵃˣ = max{α ∈ (0, 1] : xₖ + αpₖˣ ≥ (1−τⱼ)xₖ}
  //   αₖᶻ = max{α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ}
  //
  //   xₖ₊₁ = xₖ + αₖᵐᵃˣpₖˣ
  //   sₖ₊₁ = xₖ + αₖᵐᵃˣpₖˢ
  //   yₖ₊₁ = xₖ + αₖᶻpₖʸ
  //   zₖ₊₁ = xₖ + αₖᶻpₖᶻ
  //
  // [1] Nocedal, J. and Wright, S. Numerical Optimization, 2nd. ed., Ch. 19.
  //     Springer, 2006.
  // [2] http://cepac.cheme.cmu.edu/pasilectures/biegler/ipopt.pdf

  // TODO: Add problem infeasibility checks; throw exception?

  // Barrier parameter scale factor κ_μ for tolerance checks
  constexpr double kappa_epsilon = 0.1;

  // Barrier parameter scale factor κ_Σ for inequality constraint Lagrange
  // multiplier safeguard
  constexpr double kappa_sigma = 1e10;

  // Fraction-to-the-boundary rule scale factor minimum
  constexpr double tau_min = 0.995;

  // Tuning parameters for μ update
  constexpr double kappa_mu = 0.1;  // (0, 1)
  constexpr double theta_mu = 1.5;  // (1, 2)

  // Barrier parameter μ
  double mu = 1;

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

  Eigen::MatrixXd e = Eigen::VectorXd::Ones(s.rows());

  // L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ − zₖᵀ(cᵢ(x)ₖ − sₖ)
  autodiff::Variable L =
      m_f.value() - (yAD.transpose() * m_equalityConstraints -
                     zAD.transpose() * (m_inequalityConstraints - sAD))(0);

  Eigen::VectorXd step = Eigen::VectorXd::Zero(x.rows(), 1);

  SetAD(m_leaves, x);
  L.Update();

  // Error estimate E_μ
  double E_mu = 0.0;

  do {
    do {
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
      Eigen::SparseMatrix<double> H = autodiff::Hessian(L, m_leaves);

      // TODO: Write tests to ensure Regularize() works
      // Regularize(H);

      //         [∇ᵀcₑ₁(x)ₖ]
      // Aₑ(x) = [∇ᵀcₑ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcₑₘ(x)ₖ]
      Eigen::SparseMatrix<double> A_e =
          autodiff::Jacobian(m_equalityConstraints, m_leaves);

      //         [∇ᵀcᵢ₁(x)ₖ]
      // Aᵢ(x) = [∇ᵀcᵢ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcᵢₘ(x)ₖ]
      Eigen::SparseMatrix<double> A_i =
          autodiff::Jacobian(m_inequalityConstraints, m_leaves);

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

      // rhs = [∇f − Aₑᵀy + Aᵢᵀ(Σcᵢ − μS⁻¹e − z)]
      //       [               cₑ               ]
      Eigen::VectorXd c_e{m_equalityConstraints.rows()};
      for (int row = 0; row < m_equalityConstraints.rows(); row++) {
        c_e[row] = m_equalityConstraints(row).Value();
      }
      Eigen::VectorXd c_i{m_inequalityConstraints.rows()};
      for (int row = 0; row < m_inequalityConstraints.rows(); row++) {
        c_i[row] = m_inequalityConstraints(row).Value();
      }
      Eigen::VectorXd rhs{x.rows() + y.rows()};
      rhs.topRows(x.rows()) =
          Gradient(m_f.value(), m_leaves) - A_e.transpose() * y +
          A_i.transpose() * (sigma * c_i - mu * inverseS * e - z);
      rhs.bottomRows(y.rows()) = c_e;

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

      // pₖˢ = μZ⁻¹e − Σ⁻¹z − Σ⁻¹pₖᶻ
      Eigen::VectorXd p_s =
          (mu * inverseZ * e - inverseSigma * z - inverseSigma * p_z);

      // αₖᵐᵃˣ = max{α ∈ (0, 1] : xₖ + αpₖˣ ≥ (1−τⱼ)xₖ}
      double alpha_max = FractionToTheBoundaryRule(x, p_x, tau);

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
      //   zₖ₊₁⁽ⁱ⁾ = max{min{zₖ₊₁⁽ⁱ⁾, κ_Σ μⱼ/xₖ₊₁⁽ⁱ⁾}, μⱼ/(κ_Σ xₖ₊₁⁽ⁱ⁾)}
      //
      // for some fixed κ_Σ ≥ 1 after each step. See equation (16) in [2].
      for (int row = 0; row < z.rows(); ++row) {
        z(row) = std::max(std::min(z(row), kappa_sigma * mu / x(row)),
                          mu / (kappa_sigma * x(row)));
      }

      SetAD(m_leaves, x);
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

      // Update the error estimate. Based on equation (5) in [2].
      E_mu = std::max(rhs.topRows(x.rows()).lpNorm<Eigen::Infinity>() / s_d,
                      c_e.lpNorm<Eigen::Infinity>());
    } while (E_mu > kappa_epsilon * mu);

    // Update the barrier parameter.
    //
    //   μⱼ₊₁ = max{εₜₒₗ/10, min{κ_μ μⱼ, μⱼ^θ_μ}}
    //
    // See equation (7) in [2].
    mu = std::max(m_tolerance / 10.0,
                  std::min(kappa_mu * mu, std::pow(mu, theta_mu)));

    // Update the fraction-to-the-boundary rule scaling factor.
    //
    //   τⱼ = max{τₘᵢₙ, 1 − μⱼ}
    //
    // See equation (8) in [2].
    tau = std::max(tau_min, 1.0 - mu);
  } while (E_mu > m_tolerance);

  return x;
}
