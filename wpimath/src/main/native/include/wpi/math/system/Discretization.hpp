// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include <unsupported/Eigen/MatrixFunctions>

#include "wpi/math/linalg/EigenCore.hpp"
#include "wpi/units/time.hpp"

namespace wpi::math {

namespace detail {

/**
 * Returns the sum of two Eigen sizes, which is dynamic if either is.
 */
constexpr int AddSizes(int a, int b) {
  return (a == Eigen::Dynamic || b == Eigen::Dynamic) ? Eigen::Dynamic : a + b;
}

/**
 * Lazily evaluated block upper triangular matrix of the form
 *
 *   [E  F]
 *   [0  G]
 *
 * where each block is an Eigen expression. This lets linear combinations of
 * block triangular matrices fuse into one loop per block, like Eigen's
 * expression templates do for dense matrices.
 */
template <typename EExpr, typename FExpr, typename GExpr>
struct BlockTriangularExpr {
  EExpr E;
  FExpr F;
  GExpr G;
};

template <typename T>
concept BlockTriangular = requires(const T& t) {
  t.E;
  t.F;
  t.G;
};

template <BlockTriangular L, BlockTriangular R>
auto operator+(const L& lhs, const R& rhs) {
  return BlockTriangularExpr{lhs.E + rhs.E, lhs.F + rhs.F, lhs.G + rhs.G};
}

template <BlockTriangular L, BlockTriangular R>
auto operator-(const L& lhs, const R& rhs) {
  return BlockTriangularExpr{lhs.E - rhs.E, lhs.F - rhs.F, lhs.G - rhs.G};
}

template <BlockTriangular T>
auto operator*(double lhs, const T& rhs) {
  return BlockTriangularExpr{lhs * rhs.E, lhs * rhs.F, lhs * rhs.G};
}

/**
 * Block upper triangular matrix of the form
 *
 *   [E  F]
 *   [0  G]
 *
 * This structure is closed under addition, scalar multiplication, and matrix
 * multiplication.
 *
 * @tparam Rows Number of rows and columns in E, or Eigen::Dynamic.
 * @tparam Cols Number of columns in F and rows and columns in G, or
 *     Eigen::Dynamic.
 * @tparam ScalarG Whether G is stored as a scalar γ representing γI.
 */
template <int Rows, int Cols, bool ScalarG>
struct BlockTriangularMatrix {
  Matrixd<Rows, Rows> E;
  Matrixd<Rows, Cols> F;
  std::conditional_t<ScalarG, double, Matrixd<Cols, Cols>> G;

  BlockTriangularMatrix() = default;

  template <typename EExpr, typename FExpr, typename GExpr>
  BlockTriangularMatrix(const EExpr& E, const FExpr& F, const GExpr& G)
      : E(E), F(F), G(G) {}

  template <BlockTriangular T>
  BlockTriangularMatrix(const T& expr)  // NOLINT
      : E(expr.E), F(expr.F), G(expr.G) {}

  template <BlockTriangular T>
  BlockTriangularMatrix& operator=(const T& expr) {
    E = expr.E;
    F = expr.F;
    G = expr.G;
    return *this;
  }

  template <BlockTriangular T>
  BlockTriangularMatrix& operator+=(const T& expr) {
    E += expr.E;
    F += expr.F;
    G += expr.G;
    return *this;
  }

  static auto Identity(Eigen::Index rows, Eigen::Index cols) {
    if constexpr (ScalarG) {
      return BlockTriangularExpr{Matrixd<Rows, Rows>::Identity(rows, rows),
                                 Matrixd<Rows, Cols>::Zero(rows, cols), 1.0};
    } else {
      return BlockTriangularExpr{Matrixd<Rows, Rows>::Identity(rows, rows),
                                 Matrixd<Rows, Cols>::Zero(rows, cols),
                                 Matrixd<Cols, Cols>::Identity(cols, cols)};
    }
  }

  // [E₁  F₁][E₂  F₂] = [E₁E₂  E₁F₂ + F₁G₂]
  // [0   G₁][0   G₂]   [ 0       G₁G₂   ]
  BlockTriangularMatrix operator*(const BlockTriangularMatrix& rhs) const {
    BlockTriangularMatrix result;
    result.E.noalias() = E * rhs.E;
    result.F.noalias() = E * rhs.F;
    if constexpr (ScalarG) {
      result.F += rhs.G * F;
      result.G = G * rhs.G;
    } else {
      result.F.noalias() += F * rhs.G;
      result.G.noalias() = G * rhs.G;
    }
    return result;
  }

  /**
   * Returns the matrix exponential.
   *
   * This is the same scaling and squaring Padé approximant algorithm Eigen
   * uses for MatrixBase::exp(), but every intermediate matrix is block upper
   * triangular, so it operates on the nonzero blocks only.
   */
  BlockTriangularMatrix exp() const {
    // ‖X‖₁ is the max column sum
    double l1norm = E.cwiseAbs().colwise().sum().maxCoeff();
    if constexpr (ScalarG) {
      l1norm = std::max(
          l1norm,
          (F.cwiseAbs().colwise().sum().array() + std::abs(G)).maxCoeff());
    } else {
      l1norm = std::max(
          l1norm, (F.cwiseAbs().colwise().sum() + G.cwiseAbs().colwise().sum())
                      .maxCoeff());
    }

    // Padé approximant of eˣ is (V − U)⁻¹(V + U)
    BlockTriangularMatrix U;
    BlockTriangularMatrix V;
    int squarings = 0;
    if (l1norm < 1.495585217958292e-002) {
      PadeUV(*this, std::array{120.0, 60.0, 12.0, 1.0}, &U, &V);
    } else if (l1norm < 2.539398330063230e-001) {
      PadeUV(*this, std::array{30240.0, 15120.0, 3360.0, 420.0, 30.0, 1.0}, &U,
             &V);
    } else if (l1norm < 9.504178996162932e-001) {
      PadeUV(*this,
             std::array{17297280.0, 8648640.0, 1995840.0, 277200.0, 25200.0,
                        1512.0, 56.0, 1.0},
             &U, &V);
    } else if (l1norm < 2.097847961257068e+000) {
      PadeUV(*this,
             std::array{17643225600.0, 8821612800.0, 2075673600.0, 302702400.0,
                        30270240.0, 2162160.0, 110880.0, 3960.0, 90.0, 1.0},
             &U, &V);
    } else {
      constexpr double maxnorm = 5.371920351148152;
      std::frexp(l1norm / maxnorm, &squarings);
      if (squarings < 0) {
        squarings = 0;
      }
      PadeUV13(std::ldexp(1.0, -squarings) * *this, &U, &V);
    }

    // Solve (V − U)ϕ = V + U via block back substitution
    //
    //   [D₁₁  D₁₂][ϕ₁₁  ϕ₁₂] = [N₁₁  N₁₂]
    //   [ 0   D₂₂][ 0   ϕ₂₂]   [ 0   N₂₂]
    //
    //   ϕ₂₂ = D₂₂⁻¹N₂₂
    //   ϕ₁₁ = D₁₁⁻¹N₁₁
    //   ϕ₁₂ = D₁₁⁻¹(N₁₂ − D₁₂ϕ₂₂)
    const auto D = V - U;
    const auto N = V + U;
    BlockTriangularMatrix phi;
    if constexpr (ScalarG) {
      phi.G = N.G / D.G;
    } else {
      phi.G = D.G.partialPivLu().solve(N.G);
    }
    constexpr int RowsPlusCols = AddSizes(Rows, Cols);
    const Eigen::Index rows = E.rows();
    const Eigen::Index cols = F.cols();
    Matrixd<Rows, RowsPlusCols> rhs;
    rhs.resize(rows, rows + cols);
    rhs.template leftCols<Rows>(rows) = N.E;
    rhs.template rightCols<Cols>(cols) = N.F - D.F * phi.G;
    Matrixd<Rows, RowsPlusCols> sol = D.E.partialPivLu().solve(rhs);
    phi.E = sol.template leftCols<Rows>(rows);
    phi.F = sol.template rightCols<Cols>(cols);

    // Undo scaling by repeated squaring
    for (int i = 0; i < squarings; ++i) {
      phi = phi * phi;
    }

    return phi;
  }

 private:
  /**
   * Computes U and V such that (V − U)⁻¹(V + U) is the Padé approximant of eˣ
   * of degree 3, 5, 7, or 9.
   *
   * @param X Matrix argument.
   * @param b Padé coefficients.
   * @param U Storage for odd part of Padé numerator.
   * @param V Storage for even part of Padé numerator.
   */
  template <size_t N>
  static void PadeUV(const BlockTriangularMatrix& X,
                     const std::array<double, N>& b, BlockTriangularMatrix* U,
                     BlockTriangularMatrix* V) {
    const auto I = Identity(X.E.rows(), X.F.cols());
    const BlockTriangularMatrix X2 = X * X;

    BlockTriangularMatrix tmp = b[3] * X2 + b[1] * I;
    *V = b[2] * X2 + b[0] * I;

    BlockTriangularMatrix X2k = X2;
    for (size_t k = 2; 2 * k < N; ++k) {
      X2k = X2k * X2;
      tmp += b[2 * k + 1] * X2k;
      *V += b[2 * k] * X2k;
    }

    *U = X * tmp;
  }

  /**
   * Computes U and V such that (V − U)⁻¹(V + U) is the Padé approximant of eˣ
   * of degree 13.
   *
   * @param X Matrix argument.
   * @param U Storage for odd part of Padé numerator.
   * @param V Storage for even part of Padé numerator.
   */
  static void PadeUV13(const BlockTriangularMatrix& X, BlockTriangularMatrix* U,
                       BlockTriangularMatrix* V) {
    constexpr double b[] = {64764752532480000.0,
                            32382376266240000.0,
                            7771770303897600.0,
                            1187353796428800.0,
                            129060195264000.0,
                            10559470521600.0,
                            670442572800.0,
                            33522128640.0,
                            1323241920.0,
                            40840800.0,
                            960960.0,
                            16380.0,
                            182.0,
                            1.0};

    const auto I = Identity(X.E.rows(), X.F.cols());
    const BlockTriangularMatrix X2 = X * X;
    const BlockTriangularMatrix X4 = X2 * X2;
    const BlockTriangularMatrix X6 = X4 * X2;

    BlockTriangularMatrix tmp = b[13] * X6 + b[11] * X4 + b[9] * X2;
    tmp = X6 * tmp + b[7] * X6 + b[5] * X4 + b[3] * X2 + b[1] * I;
    *U = X * tmp;

    tmp = b[12] * X6 + b[10] * X4 + b[8] * X2;
    *V = X6 * tmp + b[6] * X6 + b[4] * X4 + b[2] * X2 + b[0] * I;
  }
};

}  // namespace detail

/**
 * Discretizes the given continuous A matrix.
 *
 * @tparam States Number of states.
 * @param contA Continuous system matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 */
template <int States>
void DiscretizeA(const Matrixd<States, States>& contA, wpi::units::second_t dt,
                 Matrixd<States, States>* discA) {
  // A_d = eᴬᵀ
  *discA = (contA * dt.value()).exp();
}

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @tparam States Number of states, or Eigen::Dynamic.
 * @tparam Inputs Number of inputs, or Eigen::Dynamic.
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template <int States, int Inputs>
void DiscretizeAB(const Matrixd<States, States>& contA,
                  const Matrixd<States, Inputs>& contB, wpi::units::second_t dt,
                  Matrixd<States, States>* discA,
                  Matrixd<States, Inputs>* discB) {
  // M = [A  B]
  //     [0  0]
  //
  // ϕ = eᴹᵀ = [A_d  B_d]
  //           [ 0    I ]
  const double T = dt.value();

  // For dynamic sizes, the block triangular exponential's extra heap
  // allocations outweigh its savings on small systems
  if constexpr (States == Eigen::Dynamic || Inputs == Eigen::Dynamic) {
    const Eigen::Index states = contA.rows();
    const Eigen::Index inputs = contB.cols();
    if (states + inputs < 5) {
      constexpr int Size = detail::AddSizes(States, Inputs);
      Matrixd<Size, Size> M =
          Matrixd<Size, Size>::Zero(states + inputs, states + inputs);
      M.topLeftCorner(states, states) = contA;
      M.topRightCorner(states, inputs) = contB;

      Matrixd<Size, Size> phi = (M * T).exp();

      *discA = phi.topLeftCorner(states, states);
      *discB = phi.topRightCorner(states, inputs);
      return;
    }
  }

  auto phi = detail::BlockTriangularMatrix<States, Inputs, true>{contA * T,
                                                                 contB * T, 0.0}
                 .exp();

  *discA = phi.E;
  *discB = phi.F;
}

namespace detail {

/**
 * Computes ϕ₁₂ and ϕ₂₂ of ϕ = eᴹᵀ for DiscretizeAQ() using the dense matrix
 * exponential.
 *
 * @tparam States Number of states, or Eigen::Dynamic.
 * @param contA Continuous system matrix.
 * @param Q     Symmetric continuous process noise covariance matrix.
 * @param T     Discretization timestep in seconds.
 * @param phi12 Storage for ϕ₁₂ = A_d⁻¹Q_d.
 * @param phi22 Storage for ϕ₂₂ = A_dᵀ.
 */
template <int States>
void DiscretizeAQPhiDense(const Matrixd<States, States>& contA,
                          const Matrixd<States, States>& Q, double T,
                          Matrixd<States, States>* phi12,
                          Matrixd<States, States>* phi22) {
  const Eigen::Index states = contA.rows();
  constexpr int Size = AddSizes(States, States);

  // M = [−A  Q ]
  //     [ 0  Aᵀ]
  Matrixd<Size, Size> M;
  M.resize(2 * states, 2 * states);
  M.topLeftCorner(states, states) = -contA;
  M.topRightCorner(states, states) = Q;
  M.bottomLeftCorner(states, states).setZero();
  M.bottomRightCorner(states, states) = contA.transpose();

  Matrixd<Size, Size> phi = (M * T).exp();

  *phi12 = phi.topRightCorner(states, states);
  *phi22 = phi.bottomRightCorner(states, states);
}

/**
 * Computes ϕ₁₂ and ϕ₂₂ of ϕ = eᴹᵀ for DiscretizeAQ() using the block
 * triangular matrix exponential.
 *
 * @tparam States Number of states, or Eigen::Dynamic.
 * @param contA Continuous system matrix.
 * @param Q     Symmetric continuous process noise covariance matrix.
 * @param T     Discretization timestep in seconds.
 * @param phi12 Storage for ϕ₁₂ = A_d⁻¹Q_d.
 * @param phi22 Storage for ϕ₂₂ = A_dᵀ.
 */
template <int States>
void DiscretizeAQPhiBlock(const Matrixd<States, States>& contA,
                          const Matrixd<States, States>& Q, double T,
                          Matrixd<States, States>* phi12,
                          Matrixd<States, States>* phi22) {
  // M = [−A  Q ]
  //     [ 0  Aᵀ]
  auto phi = BlockTriangularMatrix<States, States, false>{-contA * T, Q * T,
                                                          contA.transpose() * T}
                 .exp();

  *phi12 = phi.F;
  *phi22 = phi.G;
}

}  // namespace detail

/**
 * Discretizes the given continuous A and Q matrices.
 *
 * @tparam States Number of states, or Eigen::Dynamic.
 * @param contA Continuous system matrix.
 * @param contQ Continuous process noise covariance matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discQ Storage for discrete process noise covariance matrix.
 */
template <int States>
void DiscretizeAQ(const Matrixd<States, States>& contA,
                  const Matrixd<States, States>& contQ, wpi::units::second_t dt,
                  Matrixd<States, States>* discA,
                  Matrixd<States, States>* discQ) {
  // Make continuous Q symmetric if it isn't already
  Matrixd<States, States> Q = (contQ + contQ.transpose()) / 2.0;

  // M = [−A  Q ]
  //     [ 0  Aᵀ]
  //
  // ϕ = eᴹᵀ = [−A_d  A_d⁻¹Q_d]
  //           [ 0      A_dᵀ  ]
  const double T = dt.value();
  Matrixd<States, States> phi12;
  Matrixd<States, States> phi22;

  // Eigen's dense exponential is faster than the block triangular one for 2
  // states, and for dynamic sizes below 8 states since the block triangular
  // one's extra heap allocations outweigh its savings
  if constexpr (States == 2) {
    detail::DiscretizeAQPhiDense<States>(contA, Q, T, &phi12, &phi22);
  } else if constexpr (States == Eigen::Dynamic) {
    if (contA.rows() < 8) {
      detail::DiscretizeAQPhiDense<States>(contA, Q, T, &phi12, &phi22);
    } else {
      detail::DiscretizeAQPhiBlock<States>(contA, Q, T, &phi12, &phi22);
    }
  } else {
    detail::DiscretizeAQPhiBlock<States>(contA, Q, T, &phi12, &phi22);
  }

  *discA = phi22.transpose();

  Q = *discA * phi12;

  // Make discrete Q symmetric if it isn't already
  *discQ = (Q + Q.transpose()) / 2.0;
}

/**
 * Returns a discretized version of the provided continuous measurement noise
 * covariance matrix.
 *
 * @tparam Outputs Number of outputs.
 * @param R  Continuous measurement noise covariance matrix.
 * @param dt Discretization timestep.
 */
template <int Outputs>
Matrixd<Outputs, Outputs> DiscretizeR(const Matrixd<Outputs, Outputs>& R,
                                      wpi::units::second_t dt) {
  // R_d = 1/T R
  return R / dt.value();
}

}  // namespace wpi::math
