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
      vars(row, col) = m_leaves[oldSize + row * cols + col];
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

double Problem::FractionToTheBoundaryRule(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    const Eigen::Ref<const Eigen::VectorXd>& p) {
  constexpr double tau = 0.995;

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
  // L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ - zₖᵀ(cᵢ(x)ₖ - sₖ)
  // Hₖ = ∇²ₓₓL(x, s, y, z)ₖ
  //
  //     [s₁ 0 ⋯ 0 ]
  // S = [0  ⋱   ⋮ ]
  //     [⋮    ⋱ 0 ]
  //     [0  ⋯ 0 sₘ]
  //
  //     [z₁ 0 ⋯ 0 ]
  // Z = [0  ⋱   ⋮ ]
  //     [⋮    ⋱ 0 ]
  //     [0  ⋯ 0 zₘ]
  //
  // where m is the number of inequality constraints.
  //
  // Σ = S⁻¹Z
  //
  // e is a column vector of ones with a number of rows equal to the number of
  // inequality constraints.
  //
  // Let f = f(x)ₖ, H = H(x)ₖ, Aₑ = Aₑ(x)ₖ, and Aᵢ = Aᵢ(x)ₖ for clarity.
  //
  // We want to solve the Newton-KKT system shown in equation (19.16) in [1].
  //
  // [H + AᵢᵀΣAᵢ  Aₑᵀ][ pₖˣ] = [∇f − Aₑᵀy − Aᵢᵀz + Aᵢᵀ(Σcᵢ − μS⁻¹e)]
  // [    Aₑ       0 ][−pₖʸ]   [                cₑ                 ]
  //
  // TODO: Derivations for pₖᶻ and pₖˢ
  //
  // The iterate pₖᶻ = −Σcᵢ + μS⁻¹e − ΣAᵢpₖˣ.
  // The iterate pₖˢ = μZ⁻¹e − Σ⁻¹z − Σ⁻¹pₖᶻ.
  //
  // The fraction-to-the-boundary rule is used to compute αₖᵐᵃˣ and αₖᶻ. See
  // equations (15a) and (15b) in [2].
  //
  // αₖᵐᵃˣ = max{α ∈ (0, 1] : xₖ + αpₖˣ ≥ (1−τⱼ)xₖ}
  // αₖᶻ = max{α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ}
  //
  // xₖ₊₁ = xₖ + αₖᵐᵃˣpₖˣ
  // sₖ₊₁ = xₖ + αₖᵐᵃˣpₖˢ
  // yₖ₊₁ = xₖ + αₖᶻpₖʸ
  // zₖ₊₁ = xₖ + αₖᶻpₖᶻ
  //
  // [1] Nocedal, J. and Wright, S. Numerical Optimization, 2nd. ed., Ch. 18.
  //     Springer, 2006.
  // [2] http://cepac.cheme.cmu.edu/pasilectures/biegler/ipopt.pdf

  double mu = 1;

  std::vector<Eigen::Triplet<double>> triplets;

  Eigen::VectorXd x = initialGuess;

  Eigen::VectorXd s = Eigen::VectorXd::Ones(m_inequalityConstraints.size());
  Eigen::VectorXd y{m_equalityConstraints.size(), 1};
  Eigen::VectorXd z = Eigen::VectorXd::Ones(m_inequalityConstraints.size());

  autodiff::VectorXvar sAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());
  autodiff::VectorXvar yAD{m_equalityConstraints.size(), 1};
  autodiff::VectorXvar zAD =
      autodiff::VectorXvar::Ones(m_inequalityConstraints.size());

  Eigen::MatrixXd e = Eigen::VectorXd::Ones(s.rows());

  // L(x, s, y, z)ₖ = f(x)ₖ − yₖᵀcₑ(x)ₖ - zₖᵀ(cᵢ(x)ₖ - sₖ)
  autodiff::Variable L =
      m_f.value() - (yAD.transpose() * m_equalityConstraints -
                     zAD.transpose() * (m_inequalityConstraints - sAD))(0);

  Eigen::VectorXd step = Eigen::VectorXd::Zero(x.rows(), 1);

  SetAD(m_leaves, x);
  L.Update();

  // TODO: Need a convergence test
  for (int muIterations = 0; muIterations < 7; ++muIterations) {
    for (int iterations = 0; iterations < m_maxIterations; ++iterations) {
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
      Eigen::SparseMatrix<double> H = Hessian(L, m_leaves);

      //         [∇ᵀcₑ₁(x)ₖ]
      // Aₑ(x) = [∇ᵀcₑ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcₑₘ(x)ₖ]
      triplets.clear();
      for (int row = 0; row < m_equalityConstraints.rows(); ++row) {
        Eigen::RowVectorXd g =
            Gradient(m_equalityConstraints(row), m_leaves).transpose();
        for (int col = 0; col < g.cols(); ++col) {
          if (g(col) != 0.0) {
            triplets.emplace_back(row, col, g(col));
          }
        }
      }
      Eigen::SparseMatrix<double> A_e{m_equalityConstraints.rows(), x.rows()};
      A_e.setFromTriplets(triplets.begin(), triplets.end());

      //         [∇ᵀcᵢ₁(x)ₖ]
      // Aᵢ(x) = [∇ᵀcᵢ₂(x)ₖ]
      //         [    ⋮    ]
      //         [∇ᵀcᵢₘ(x)ₖ]
      triplets.clear();
      for (int row = 0; row < m_inequalityConstraints.rows(); ++row) {
        Eigen::RowVectorXd g =
            Gradient(m_inequalityConstraints(row), m_leaves).transpose();
        for (int col = 0; col < g.cols(); ++col) {
          if (g(col) != 0.0) {
            triplets.emplace_back(row, col, g(col));
          }
        }
      }
      Eigen::SparseMatrix<double> A_i{m_inequalityConstraints.rows(), x.rows()};
      A_i.setFromTriplets(triplets.begin(), triplets.end());

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

      // rhs = [∇f − Aₑᵀy − Aᵢᵀz + Aᵢᵀ(Σcᵢ − μS⁻¹e)]
      //       [                cₑ                 ]
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
          Gradient(m_f.value(), m_leaves) - A_e.transpose() * y -
          A_i.transpose() * z +
          A_i.transpose() * (sigma * c_i - mu * inverseS * e);
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
      double alpha_max = FractionToTheBoundaryRule(x, p_x);

      // αₖᶻ = max{α ∈ (0, 1] : zₖ + αpₖᶻ ≥ (1−τⱼ)zₖ}
      double alpha_z = FractionToTheBoundaryRule(z, p_z);

      // xₖ₊₁ = xₖ + αₖᵐᵃˣpₖˣ
      // sₖ₊₁ = xₖ + αₖᵐᵃˣpₖˢ
      // yₖ₊₁ = xₖ + αₖᶻpₖʸ
      // zₖ₊₁ = xₖ + αₖᶻpₖᶻ
      x += alpha_max * p_x;
      s += alpha_max * p_s;
      y += alpha_z * p_y;
      z += alpha_z * p_z;

      SetAD(m_leaves, x);
      SetAD(sAD, s);
      SetAD(yAD, y);
      SetAD(zAD, z);
      L.Update();
    }

    mu *= 0.1;
  }

  return x;
}
