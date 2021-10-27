// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <complex>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <wpi/SymbolExports.h>
#include <wpi/expected>

namespace frc {

/**
 * Errors the continuous Lyapunov equation solver can encounter.
 */
enum class ContinuousLyapunovError {
  /// Q was not symmetric.
  QNotSymmetric,
  /// Solution was not unique.
  SolutionNotUnique,
  /// Schur factorization failed.
  SchurFactorizationFailed,
};

namespace detail {

inline Eigen::Vector<double, 1> Solve1x1ContinuousLyapunov(
    const Eigen::Vector<double, 1>& A, const Eigen::Vector<double, 1>& Q) {
  return Eigen::Vector<double, 1>{-Q(0) / (2 * A(0))};
}

inline Eigen::Matrix2d Solve2x2ContinuousLyapunov(const Eigen::Matrix2d& A,
                                                  const Eigen::Matrix2d& Q) {
  // Rewrite AᵀX + XA + Q = 0 (case where A,Q are 2x2) into linear set
  // A_vec * vec(X) = -vec(Q) and solve for X.
  //
  // Only the upper triangular part of Q is used, thus it is treated to be
  // symmetric.
  Eigen::Matrix3d A_vec{{2 * A(0, 0), 2 * A(1, 0), 0},
                        {A(0, 1), A(0, 0) + A(1, 1), A(1, 0)},
                        {0, 2 * A(0, 1), 2 * A(1, 1)}};
  const Eigen::Vector3d Q_vec{{-Q(0, 0)}, {-Q(0, 1)}, {-Q(1, 1)}};

  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for
  // quick overview of possible solver algorithms. ColPivHouseholderQR is
  // accurate and fast for small systems.
  Eigen::Vector3d x = A_vec.colPivHouseholderQr().solve(Q_vec);
  return Eigen::Matrix2d{{x(0), x(1)}, {x(1), x(2)}};
}

template <int Rows>
  requires(Rows >= 1)
Eigen::Matrix<double, Rows, Rows> SolveReducedContinuousLyapunov(
    const Eigen::Matrix<double, Rows, Rows>& S,
    const Eigen::Matrix<double, Rows, Rows>& Q_bar) {
  // Notation & partition adapted from SB03MD:
  //
  //   S = [s₁₁  sᵀ]  Q̅ = [q̅₁₁  q̅ᵀ]  X̅ = [x̅₁₁  x̅ᵀ]
  //       [0    S₁]      [q̅    Q₁]      [x̅    X₁]
  //
  // where s₁₁ is a 1x1 or 2x2 matrix.
  //
  //   s₁₁ᵀx̅₁₁ + x̅₁₁s₁₁ = -q̅₁₁              (3.1)
  //
  //   S₁ᵀx̅ + x̅s₁₁      = -q̅₁₁ - sx̅₁₁       (3.2)
  //
  //   S₁ᵀX₁ + X₁S₁     = -Q₁ - sx̅ᵀ - x̅sᵀ   (3.3)

  if constexpr (Rows == 1) {
    return Solve1x1ContinuousLyapunov(S, Q_bar);
  } else if constexpr (Rows == 2) {
    return Solve2x2ContinuousLyapunov(S, Q_bar);
  } else if (std::abs(S(1, 0)) <
             5 * std::numeric_limits<double>::epsilon() * S.norm()) {
    // Top-left block of S is 1x1

    const Eigen::Matrix<double, 1, 1> s_11 = S.topLeftCorner(1, 1);

    const Eigen::Matrix<double, Rows - 1, 1> s =
        S.transpose().bottomLeftCorner(Rows - 1, 1);

    const Eigen::Matrix<double, Rows - 1, Rows - 1> S_1 =
        S.bottomRightCorner(Rows - 1, Rows - 1);

    const Eigen::Matrix<double, 1, 1> q_bar_11 = Q_bar.topLeftCorner(1, 1);

    const Eigen::Matrix<double, Rows - 1, 1> q_bar =
        Q_bar.transpose().bottomLeftCorner(Rows - 1, 1);

    const Eigen::Matrix<double, Rows - 1, Rows - 1> Q_1 =
        Q_bar.bottomRightCorner(Rows - 1, Rows - 1);

    // Solving equation 3.1
    Eigen::Matrix<double, 1, 1> x_bar_11 =
        Solve1x1ContinuousLyapunov(s_11, q_bar_11);

    // Solving equation 3.2
    const Eigen::Matrix<double, Rows - 1, Rows - 1> lhs =
        S_1.transpose() +
        Eigen::Matrix<double, S_1.cols(), S_1.rows()>::Identity() * s_11(0);
    const Eigen::Matrix<double, Rows - 1, 1> rhs = -q_bar - s * x_bar_11(0);
    Eigen::Matrix<double, Rows - 1, 1> x_bar =
        lhs.colPivHouseholderQr().solve(rhs);

    // Set up equation 3.3
    const Eigen::Matrix<double, Rows - 1, Rows - 1> temp_summand =
        s * x_bar.transpose();

    // Since Q₁ is only an upper triangular matrix, with NAN in the lower part,
    // Q_NEW_ᵤₚₚₑᵣ is so as well.
    const Eigen::Matrix<double, Rows - 1, Rows - 1> Q_new_upper =
        (Q_1 + temp_summand + temp_summand.transpose());

    Eigen::Matrix<double, Rows, Rows> X_bar;
    X_bar << x_bar_11, x_bar.transpose(), x_bar,
        SolveReducedContinuousLyapunov(S_1, Q_new_upper);

    return X_bar;
  } else {
    // Top-left block of S is 2x2

    const Eigen::Matrix2d s_11 = S.topLeftCorner(2, 2);

    const Eigen::Matrix<double, Rows - 2, 2> s =
        S.transpose().bottomLeftCorner(Rows - 2, 2);

    const Eigen::Matrix<double, Rows - 2, Rows - 2> S_1 =
        S.bottomRightCorner(Rows - 2, Rows - 2);

    const Eigen::Matrix2d q_bar_11 = Q_bar.topLeftCorner(2, 2);

    const Eigen::Matrix<double, Rows - 2, 2> q_bar =
        Q_bar.transpose().bottomLeftCorner(Rows - 2, 2);

    const Eigen::Matrix<double, Rows - 2, Rows - 2> Q_1 =
        Q_bar.bottomRightCorner(Rows - 2, Rows - 2);

    Eigen::Matrix2d x_bar_11 = Solve2x2ContinuousLyapunov(s_11, q_bar_11);

    // Solving equation 3.2
    //
    // The equation reads as
    //
    //   S₁ᵀx̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁
    //
    // where S₁ ∈ ℝᵐ⁻²ˣᵐ⁻²; x̅, q̅, s ∈ ℝᵐ⁻²ˣ² and s₁₁, x̅₁₁ ∈ ℝ²ˣ².
    //
    // We solve the linear equation by vectorization and feeding it into
    // colPivHouseHolderQr().
    //
    // Notation:
    //
    // The elements in s₁₁ are names as the following:
    //
    //   s₁₁ = [s₁₁(0,0) s₁₁(0,1); s₁₁(1,0) s₁₁(1,1)]
    //
    // Note that eigen starts counting at 0, and not as above at 1.
    //
    // The ith column of a matrix is accessed by [i], i.e. the first column of x̅
    // is x̅[0].
    //
    // Define:
    //
    //   S_1_vec = [S₁ᵀ   0ₘₓₘ]
    //             [0ₘₓₘ  S₁ᵀ ]
    //
    //   S_11_vec = [s₁₁(0,0)Iₘₓₘ  s₁₁(1,0)Iₘₓₘ]
    //              [s₁₁(0,1)Iₘₓₘ  s₁₁(1,1)Iₘₓₘ]
    //
    // Now define lhs = S_1_vec + S_11_vec.
    //
    //   x_vec = [x̅[0]ᵀ  x̅[1]ᵀ]ᵀ where [i] is the ith column
    //
    // rhs = -[(q̅ + sx̅₁₁)[0]ᵀ  (q̅ + sx̅₁₁)[1]ᵀ]ᵀ
    //
    // Now S₁ᵀx̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁ can be rewritten into:
    //
    //   lhs * x_vec = rhs
    //
    // This expression can be fed into colPivHouseHolderQr() and solved.
    // Finally, construct x_bar from x_vec.

    Eigen::Matrix<double, 2 * (Rows - 2), 2 * (Rows - 2)> S_1_vec;
    S_1_vec << S_1.transpose(),
        Eigen::Matrix<double, Rows - 2, Rows - 2>::Zero(),
        Eigen::Matrix<double, Rows - 2, Rows - 2>::Zero(), S_1.transpose();

    Eigen::Matrix<double, 2 * (Rows - 2), 2 * (Rows - 2)> s_11_vec;
    s_11_vec << s_11(0, 0) *
                    Eigen::Matrix<double, Rows - 2, Rows - 2>::Identity(),
        s_11(1, 0) * Eigen::Matrix<double, Rows - 2, Rows - 2>::Identity(),
        s_11(0, 1) * Eigen::Matrix<double, Rows - 2, Rows - 2>::Identity(),
        s_11(1, 1) * Eigen::Matrix<double, Rows - 2, Rows - 2>::Identity();
    Eigen::Matrix<double, 2 * (Rows - 2), 2 * (Rows - 2)> lhs =
        S_1_vec + s_11_vec;

    Eigen::Vector<double, 2 * (Rows - 2)> rhs;
    rhs << (-q_bar - s * x_bar_11).col(0), (-q_bar - s * x_bar_11).col(1);

    Eigen::Vector<double, rhs.rows()> x_bar_vec =
        lhs.colPivHouseholderQr().solve(rhs);
    Eigen::Matrix<double, Rows - 2, 2> x_bar;
    x_bar.col(0) = x_bar_vec.segment(0, Rows - 2);
    x_bar.col(1) = x_bar_vec.segment(Rows - 2, Rows - 2);

    // Set up equation 3.3
    const Eigen::Matrix<double, Rows - 2, Rows - 2> temp_summand =
        s * x_bar.transpose();

    // Since Q₁ is only an upper triangular matrix, with NAN in the lower
    // part, Q_NEW_ᵤₚₚₑᵣ is so as well.
    const Eigen::Matrix<double, Rows - 2, Rows - 2> Q_new_upper =
        (Q_1 + temp_summand + temp_summand.transpose());

    Eigen::Matrix<double, Rows, Rows> X_bar;
    X_bar << x_bar_11, x_bar.transpose(), x_bar,
        SolveReducedContinuousLyapunov(S_1, Q_new_upper);

    return X_bar;
  }
}

}  // namespace detail

/**
 * Computes a unique solution X to the continuous Lyapunov equation
 * AᵀX + XA + Q = 0.
 *
 * Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists a unique
 * solution if and only if λᵢ + λ̅ⱼ ≠ 0 ∀ i,j, where λ̅ⱼ is the complex conjugate
 * of λⱼ. There are no further limitations on the eigenvalues of A.
 *
 * Returns failure if the solution is not unique.
 *
 * Further, if all λᵢ have negative real parts, and if Q is positive
 * semi-definite, then X is also positive semi-definite [1]. Therefore, if one
 * searches for a Lyapunov function V(z) = zᵀXz for the stable linear system
 * ż = Az, then the solution of the Lyapunov equation AᵀX + XA + Q = 0 only
 * returns a valid Lyapunov function if Q is positive semi-definite.
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * transformation Q = -C. The complexity of this routine is O(n³). If A is
 * larger than 2x2, then a Schur factorization is performed.
 *
 * Returns failure if Schur factorization failed.
 *
 * A tolerance of ε is used to check if a double variable is equal to zero,
 * where the default value for ε is 1e-10. It has been used to check (1) if λᵢ +
 * λ̅ⱼ = 0, ∀ i,j; (2) if A is a 1x1 zero matrix; (3) if A's trace or determinant
 * is 0 when A is a 2x2 matrix.
 *
 * [1] Bartels, R.H. and G.W. Stewart, "Solution of the Matrix Equation AX + XB
 *     = C," Comm. of the ACM, Vol. 15, No. 9, 1972.
 *
 * [2] http://slicot.org/objects/software/shared/doc/SB03MD.html
 *
 * @param A A square matrix.
 * @param Q A symmetric matrix.
 * @return Solution to the continuous Lyapunov equation on success, or
 *   ContinuousLyapunovError on failure.
 */
template <int Rows>
  requires(Rows >= 1)
wpi::expected<Eigen::Matrix<double, Rows, Rows>, ContinuousLyapunovError>
ContinuousLyapunov(const Eigen::Matrix<double, Rows, Rows>& A,
                   const Eigen::Matrix<double, Rows, Rows>& Q) {
  auto isZero = [](double x) { return std::abs(x) < 1e-10; };

  if ((Q - Q.transpose()).norm() > 1e-10) {
    return wpi::unexpected{ContinuousLyapunovError::QNotSymmetric};
  }

  if constexpr (Rows == 1) {
    if (isZero(A(0, 0))) {
      return wpi::unexpected{ContinuousLyapunovError::SolutionNotUnique};
    }
    return detail::Solve1x1ContinuousLyapunov(A, Q);
  } else if constexpr (Rows == 2) {
    // If A has two real eigenvalues λ₁ and λ₂, then we check:
    //
    //   1. If λ₁ or λ₂ is 0, i.e. if the determinant of A is 0
    //   2. If λ₁ + λ₂ = 0, i.e., if the trace of A is 0
    //
    // If A has two complex eigenvalues λ₁ and λ̅₁, then we check if λ₁ + λ̅₁ = 0,
    // i.e., if the trace of A is 0.
    if (isZero(A(0, 0) + A(1, 1)) ||
        isZero(A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1))) {
      return wpi::unexpected{ContinuousLyapunovError::SolutionNotUnique};
    }

    // Reducing Q to upper triangular form
    Eigen::Matrix<double, Rows, Rows> Q_upper = Q;
    Q_upper(1, 0) = NAN;
    return detail::Solve2x2ContinuousLyapunov(A, Q_upper);
  } else {
    Eigen::Vector<std::complex<double>, Rows> eig_val = A.eigenvalues();
    for (int i = 0; i < eig_val.size(); ++i) {
      for (int j = i; j < eig_val.size(); ++j) {
        std::complex<double> eig_sum = eig_val(i) + std::conj(eig_val(j));
        if (isZero(eig_sum.real()) && isZero(eig_sum.imag())) {
          return wpi::unexpected{ContinuousLyapunovError::SolutionNotUnique};
        }
      }
    }

    // Transform into reduced form
    //
    //   SᵀX̅ + X̅S = -Q̅
    //
    // where
    //
    //   A = USUᵀ
    //   X̅ = UᵀXU
    //   Q̅ = UᵀQU
    //   U is the unitary matrix
    //   S the reduced form
    //
    // Note that the expression for X̅ in [2] is incorrect.
    Eigen::RealSchur<Eigen::Matrix<double, Rows, Rows>> schur{A};
    if (schur.info() != Eigen::Success) {
      return wpi::unexpected{ContinuousLyapunovError::SchurFactorizationFailed};
    }
    const auto& U = schur.matrixU();
    const auto& S = schur.matrixT();

    // Reduce the symmetric Q̅ to its upper triangular form Q̅ᵤₚₚₑᵣ
    Eigen::Matrix<double, Rows, Rows> Q_bar_upper =
        Eigen::Matrix<double, Rows, Rows>::Constant(NAN);
    Q_bar_upper.template triangularView<Eigen::Upper>() = U.transpose() * Q * U;
    return (U * detail::SolveReducedContinuousLyapunov(S, Q_bar_upper) *
            U.transpose());
  }
}

}  // namespace frc
