// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.linalg;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.hessenberg.HessenbergSimilarDecomposition_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.util.Complex;

/**
 * Computes real matrices raised to an arbitrary real power with the Schur–Padé algorithm.
 *
 * <p>This is a port of Eigen's MatrixPower class.
 *
 * <p>[1] N. J. Higham and L. Lin, "A Schur–Padé Algorithm for Fractional Powers of a Matrix," SIAM
 * Journal on Matrix Analysis and Applications, vol. 32, no. 3, pp. 1056–1078, Jul. 2011. DOI:
 * 10.1137/10081232X
 */
final class MatrixPower {
  private MatrixPower() {
    // Utility class.
  }

  /**
   * Givens rotation G = [c, conj(s); -s, conj(c)] such that applying it to the left of [p; q] gives
   * [r; 0].
   */
  private static final class Givens {
    final Complex c;
    final Complex s;
    final Complex r;

    /**
     * Constructs a Givens rotation.
     *
     * <p>This implements the continuous Givens rotation generation algorithm found in Anderson
     * (2000), Discontinuous Plane Rotations and the Symmetric Eigenvalue Problem. LAPACK Working
     * Note 150, University of Tennessee, UT-CS-00-454, December 4, 2000.
     *
     * @param p First element.
     * @param q Second element.
     */
    Givens(Complex p, Complex q) {
      if (q.isZero()) {
        c = new Complex(p.getReal() < 0.0 ? -1.0 : 1.0, 0.0);
        s = Complex.ZERO;
        r = c.times(p);
      } else if (p.isZero()) {
        c = Complex.ZERO;
        s = q.negate().div(q.abs());
        r = new Complex(q.abs(), 0.0);
      } else {
        double p1 = p.norm1();
        double q1 = q.norm1();
        if (p1 >= q1) {
          var ps = p.div(p1);
          double p2 = ps.abs2();
          var qs = q.div(p1);
          double q2 = qs.abs2();

          double u = Math.sqrt(1.0 + q2 / p2);
          if (p.getReal() < 0.0) {
            u = -u;
          }

          c = new Complex(1.0 / u, 0.0);
          s = qs.negate().times(ps.conj()).times(c.getReal() / p2);
          r = p.times(u);
        } else {
          var ps = p.div(q1);
          double p2 = ps.abs2();
          var qs = q.div(q1);
          double q2 = qs.abs2();

          double u = q1 * Math.sqrt(p2 + q2);
          if (p.getReal() < 0.0) {
            u = -u;
          }

          p1 = p.abs();
          ps = p.div(p1);
          c = new Complex(p1 / u, 0.0);
          s = ps.conj().negate().times(q.div(u));
          r = ps.times(u);
        }
      }
    }
  }

  /**
   * Applies the plane rotation [c, conj(s); -s, conj(c)] to the vectors x and y, where x and y are
   * rows p and q of M (or columns p and q if applying to columns).
   *
   * @param M Matrix.
   * @param p Index of x.
   * @param q Index of y.
   * @param c Rotation's c.
   * @param s Rotation's s.
   * @param begin First index along the vectors.
   * @param end One past the last index along the vectors.
   * @param rows True if x and y are rows of M, false if they are columns.
   */
  private static void applyRotation(
      Complex[][] M, int p, int q, Complex c, Complex s, int begin, int end, boolean rows) {
    if (c.equals(Complex.ONE) && s.isZero()) {
      return;
    }

    for (int k = begin; k < end; ++k) {
      var x = rows ? M[p][k] : M[k][p];
      var y = rows ? M[q][k] : M[k][q];
      var newX = c.times(x).plus(s.conj().times(y));
      var newY = s.negate().times(x).plus(c.conj().times(y));
      if (rows) {
        M[p][k] = newX;
        M[q][k] = newY;
      } else {
        M[k][p] = newX;
        M[k][q] = newY;
      }
    }
  }

  /**
   * Computes M = GᴴM where G is the given rotation in the plane of rows p and q.
   *
   * @param M Matrix.
   * @param p First row.
   * @param q Second row.
   * @param rot Rotation.
   * @param colBegin First column to rotate.
   * @param colEnd One past the last column to rotate.
   */
  private static void applyAdjointOnTheLeft(
      Complex[][] M, int p, int q, Givens rot, int colBegin, int colEnd) {
    applyRotation(M, p, q, rot.c.conj(), rot.s.negate(), colBegin, colEnd, true);
  }

  /**
   * Computes M = MG where G is the given rotation in the plane of columns p and q.
   *
   * @param M Matrix.
   * @param p First column.
   * @param q Second column.
   * @param rot Rotation.
   * @param rowBegin First row to rotate.
   * @param rowEnd One past the last row to rotate.
   */
  private static void applyOnTheRight(
      Complex[][] M, int p, int q, Givens rot, int rowBegin, int rowEnd) {
    applyRotation(M, p, q, rot.c, rot.s.conj().negate(), rowBegin, rowEnd, false);
  }

  private static Complex[][] zeros(int rows, int cols) {
    var M = new Complex[rows][cols];
    for (var row : M) {
      java.util.Arrays.fill(row, Complex.ZERO);
    }
    return M;
  }

  private static Complex[][] identity(int n) {
    var I = zeros(n, n);
    for (int i = 0; i < n; ++i) {
      I[i][i] = Complex.ONE;
    }
    return I;
  }

  private static Complex[][] toComplex(DMatrixRMaj A) {
    var M = new Complex[A.getNumRows()][A.getNumCols()];
    for (int row = 0; row < A.getNumRows(); ++row) {
      for (int col = 0; col < A.getNumCols(); ++col) {
        M[row][col] = new Complex(A.get(row, col), 0.0);
      }
    }
    return M;
  }

  /** Returns the upper triangular part of A. */
  private static Complex[][] triu(Complex[][] A) {
    var M = zeros(A.length, A[0].length);
    for (int row = 0; row < A.length; ++row) {
      for (int col = row; col < A[0].length; ++col) {
        M[row][col] = A[row][col];
      }
    }
    return M;
  }

  private static Complex[][] times(Complex[][] A, Complex[][] B) {
    var M = zeros(A.length, B[0].length);
    for (int row = 0; row < A.length; ++row) {
      for (int k = 0; k < B.length; ++k) {
        var a = A[row][k];
        if (a.isZero()) {
          continue;
        }
        for (int col = 0; col < B[0].length; ++col) {
          M[row][col] = M[row][col].plus(a.times(B[k][col]));
        }
      }
    }
    return M;
  }

  private static Complex[][] times(Complex[][] A, double s) {
    var M = new Complex[A.length][A[0].length];
    for (int row = 0; row < A.length; ++row) {
      for (int col = 0; col < A[0].length; ++col) {
        M[row][col] = A[row][col].times(s);
      }
    }
    return M;
  }

  /** Returns the conjugate transpose of A. */
  private static Complex[][] adjoint(Complex[][] A) {
    var M = new Complex[A[0].length][A.length];
    for (int row = 0; row < A.length; ++row) {
      for (int col = 0; col < A[0].length; ++col) {
        M[col][row] = A[row][col].conj();
      }
    }
    return M;
  }

  /**
   * Solves RX = B for X via back substitution, where R is the upper triangular part of the given
   * matrix.
   *
   * @param R Matrix whose upper triangular part is used.
   * @param B Right-hand side.
   * @return X.
   */
  private static Complex[][] solveUpperTriangular(Complex[][] R, Complex[][] B) {
    int n = R.length;
    var X = zeros(n, B[0].length);
    for (int col = 0; col < B[0].length; ++col) {
      for (int i = n - 1; i >= 0; --i) {
        var sum = B[i][col];
        for (int k = i + 1; k < n; ++k) {
          sum = sum.minus(R[i][k].times(X[k][col]));
        }
        X[i][col] = sum.div(R[i][i]);
      }
    }
    return X;
  }

  /** Returns the induced 1-norm (max absolute column sum) of A. */
  private static double normIndP1(Complex[][] A) {
    double norm = 0.0;
    for (int col = 0; col < A[0].length; ++col) {
      double sum = 0.0;
      for (var row : A) {
        sum += row[col].abs();
      }
      norm = Math.max(norm, sum);
    }
    return norm;
  }

  /**
   * Complex Schur decomposition A = UTUᴴ of a real square matrix A where U is unitary and T is
   * upper triangular.
   *
   * <p>This is a port of Eigen's ComplexSchur class.
   */
  private static final class ComplexSchur {
    private static final int MAX_ITERATIONS_PER_ROW = 30;

    final Complex[][] T;
    final Complex[][] U;

    ComplexSchur(DMatrixRMaj A) {
      int n = A.getNumRows();
      if (n == 1) {
        T = toComplex(A);
        U = identity(1);
        return;
      }

      // Reduce A to Hessenberg form A = QHQᵀ
      var hess = new HessenbergSimilarDecomposition_DDRM(n);
      hess.decompose(A.copy());
      T = toComplex(hess.getH(null));
      U = toComplex(hess.getQ(null));

      reduceToTriangularForm();
    }

    /**
     * If T(i + 1, i) is negligible in floating point arithmetic compared to T(i, i) and T(i + 1, i
     * + 1), then set it to zero and return true, else return false.
     */
    private boolean subdiagonalEntryIsNegligible(int i) {
      double d = T[i][i].norm1() + T[i + 1][i + 1].norm1();
      double sd = T[i + 1][i].norm1();
      if (sd <= d * Math.ulp(1.0)) {
        T[i + 1][i] = Complex.ZERO;
        return true;
      }
      return false;
    }

    /** Computes the shift in the current QR iteration. */
    private Complex computeShift(int iu, int iter) {
      if ((iter == 10 || iter == 20) && iu > 1) {
        // Exceptional shift, taken from http://www.netlib.org/eispack/comqr.f
        return new Complex(
            Math.abs(T[iu][iu - 1].getReal()) + Math.abs(T[iu - 1][iu - 2].getReal()), 0.0);
      }

      // Compute the shift as one of the eigenvalues of t, the 2x2 diagonal block on the bottom of
      // the active submatrix. The normalization by normt is to avoid under/overflow.
      double normt =
          T[iu - 1][iu - 1].abs() + T[iu - 1][iu].abs() + T[iu][iu - 1].abs() + T[iu][iu].abs();
      var t00 = T[iu - 1][iu - 1].div(normt);
      var t01 = T[iu - 1][iu].div(normt);
      var t10 = T[iu][iu - 1].div(normt);
      var t11 = T[iu][iu].div(normt);

      var b = t01.times(t10);
      var c = t00.minus(t11);
      var disc = c.times(c).plus(b.times(4.0)).sqrt();
      var det = t00.times(t11).minus(b);
      var trace = t00.plus(t11);
      var eival1 = trace.plus(disc).div(2.0);
      var eival2 = trace.minus(disc).div(2.0);
      double eival1Norm = eival1.norm1();
      double eival2Norm = eival2.norm1();
      // A division by zero can only occur if eival1 == eival2 == 0. In this case, det == 0, and
      // all we have to do is check that eival2Norm != 0.
      if (eival1Norm > eival2Norm) {
        eival2 = det.div(eival1);
      } else if (eival2Norm != 0.0) {
        eival1 = det.div(eival2);
      }

      // Choose the eigenvalue closest to the bottom entry of the diagonal
      if (eival1.minus(t11).norm1() < eival2.minus(t11).norm1()) {
        return eival1.times(normt);
      } else {
        return eival2.times(normt);
      }
    }

    /** Reduces the Hessenberg matrix T to triangular form by QR iteration. */
    private void reduceToTriangularForm() {
      int n = T.length;
      int maxIters = MAX_ITERATIONS_PER_ROW * n;

      // T is divided in three parts:
      //
      //   * Rows 0, …, il - 1 are decoupled from the rest because T(il, il - 1) is zero.
      //   * Rows il, …, iu is the part we are working on (the active submatrix).
      //   * Rows iu + 1, …, n - 1 are already brought in triangular form.
      int iu = n - 1;
      int iter = 0; // Number of iterations we are working on the (iu, iu) element
      int totalIter = 0; // Number of iterations for whole matrix

      while (true) {
        // Find iu, the bottom row of the active submatrix
        while (iu > 0) {
          if (!subdiagonalEntryIsNegligible(iu - 1)) {
            break;
          }
          iter = 0;
          --iu;
        }

        // If iu is zero, the whole matrix is triangularized
        if (iu == 0) {
          break;
        }

        // If we spent too many iterations, give up
        ++iter;
        ++totalIter;
        if (totalIter > maxIters) {
          break;
        }

        // Find il, the top row of the active submatrix
        int il = iu - 1;
        while (il > 0 && !subdiagonalEntryIsNegligible(il - 1)) {
          --il;
        }

        // Perform the QR step using Givens rotations. The first rotation creates a bulge; the
        // (il + 2, il) element becomes nonzero. This bulge is chased down to the bottom of the
        // active submatrix.
        var shift = computeShift(iu, iter);
        var rot = new Givens(T[il][il].minus(shift), T[il + 1][il]);
        applyAdjointOnTheLeft(T, il, il + 1, rot, il, n);
        applyOnTheRight(T, il, il + 1, rot, 0, Math.min(il + 2, iu) + 1);
        applyOnTheRight(U, il, il + 1, rot, 0, n);

        for (int i = il + 1; i < iu; ++i) {
          rot = new Givens(T[i][i - 1], T[i + 1][i - 1]);
          T[i][i - 1] = rot.r;
          T[i + 1][i - 1] = Complex.ZERO;
          applyAdjointOnTheLeft(T, i, i + 1, rot, i, n);
          applyOnTheRight(T, i, i + 1, rot, 0, Math.min(i + 2, iu) + 1);
          applyOnTheRight(U, i, i + 1, rot, 0, n);
        }
      }
    }
  }

  /**
   * Computes the square root of an upper triangular matrix.
   *
   * @param T Upper triangular matrix.
   * @return Upper triangular square root of T.
   */
  private static Complex[][] sqrtTriangular(Complex[][] T) {
    // [1] Å. Björck and S. Hammarling, "A Schur method for the square root of a matrix," Linear
    //     Algebra and its Applications, vol. 52–53, pp. 127–140, Jul. 1983.
    //     DOI: 10.1016/0024-3795(83)80010-X

    int n = T.length;
    var B = zeros(n, n);
    for (int i = 0; i < n; ++i) {
      B[i][i] = T[i][i].sqrt();
    }

    // bᵢⱼ = (tᵢⱼ − Σₖ bᵢₖbₖⱼ)/(bᵢᵢ + bⱼⱼ) for k = i + 1, …, j − 1
    for (int j = 1; j < n; ++j) {
      for (int i = j - 1; i >= 0; --i) {
        var tmp = Complex.ZERO;
        for (int k = i + 1; k < j; ++k) {
          tmp = tmp.plus(B[i][k].times(B[k][j]));
        }
        // Denominator may be zero if the original matrix is singular
        B[i][j] = T[i][j].minus(tmp).div(B[i][i].plus(B[j][j]));
      }
    }

    return B;
  }

  /**
   * Returns the degree of the Padé approximant needed for the given ‖I − T‖₁.
   *
   * @param normIminusT ‖I − T‖₁.
   * @return The Padé approximant degree.
   */
  private static int getPadeDegree(double normIminusT) {
    // Max norms for Padé approximants with degrees 3 through 7
    final double[] maxNormForPade = {
      1.884160592658218e-2,
      6.038881904059573e-2,
      1.239917516308172e-1,
      1.999045567181744e-1,
      2.789358995219730e-1
    };
    int degree = 3;
    while (degree <= 7 && normIminusT > maxNormForPade[degree - 3]) {
      ++degree;
    }
    return degree;
  }

  /**
   * Returns the [m/m] Padé approximant of (I − (I − T))ᵖ = Tᵖ evaluated as a continued fraction.
   * See Algorithm 4.1 of [1].
   *
   * @param degree Padé approximant degree m.
   * @param IminusT I − T where T is an upper triangular matrix.
   * @param p Exponent.
   * @return Approximation of Tᵖ.
   */
  private static Complex[][] computePade(int degree, Complex[][] IminusT, double p) {
    int n = IminusT.length;

    int i = 2 * degree;
    var result = times(IminusT, (p - degree) / (2 * i - 2));

    for (--i; i > 0; --i) {
      double coeff;
      if (i == 1) {
        coeff = -p;
      } else {
        // The continued fraction's coefficients are indexed by k = ⌊i/2⌋
        int k = i / 2;
        if ((i & 1) != 0) {
          coeff = (-p - k) / (2 * i);
        } else {
          coeff = (p - k) / (2 * i - 2);
        }
      }

      // result = (I + result) \ (coeff (I − T))
      var IplusResult = triu(result);
      for (int k = 0; k < n; ++k) {
        IplusResult[k][k] = IplusResult[k][k].plus(1.0);
      }
      result = solveUpperTriangular(IplusResult, times(IminusT, coeff));
    }

    for (int k = 0; k < n; ++k) {
      result[k][k] = result[k][k].plus(1.0);
    }
    return result;
  }

  /**
   * Recomputes the diagonal and superdiagonal of Tᵖ directly from T for accuracy. See Section 5 of
   * [1].
   *
   * @param T Upper triangular matrix.
   * @param result Storage for Tᵖ. Only the diagonal and superdiagonal are written.
   * @param p Exponent.
   */
  private static void compute2x2(Complex[][] T, Complex[][] result, double p) {
    result[0][0] = T[0][0].pow(p);

    for (int i = 1; i < T.length; ++i) {
      result[i][i] = T[i][i].pow(p);
      if (T[i - 1][i - 1].equals(T[i][i])) {
        result[i - 1][i] = T[i][i].pow(p - 1).times(p);
      } else if (2.0 * T[i - 1][i - 1].abs() < T[i][i].abs()
          || 2.0 * T[i][i].abs() < T[i - 1][i - 1].abs()) {
        result[i - 1][i] =
            result[i][i].minus(result[i - 1][i - 1]).div(T[i][i].minus(T[i - 1][i - 1]));
      } else {
        result[i - 1][i] = computeSuperDiag(T[i][i], T[i - 1][i - 1], p);
      }
      result[i - 1][i] = result[i - 1][i].times(T[i - 1][i]);
    }
  }

  /**
   * Computes the divided difference (curr^p − prevᵖ)/(curr − prev) without catastrophic
   * cancellation. See equation (5.6) of [1].
   */
  private static Complex computeSuperDiag(Complex curr, Complex prev, double p) {
    var logCurr = curr.log();
    var logPrev = prev.log();
    double unwindingNumber =
        Math.ceil((logCurr.minus(logPrev).getImag() - Math.PI) / (2.0 * Math.PI));
    var w =
        curr.minus(prev)
            .div(prev)
            .log1p()
            .div(2.0)
            .plus(new Complex(0.0, Math.PI * unwindingNumber));
    return logCurr
        .plus(logPrev)
        .times(0.5 * p)
        .exp()
        .times(2.0)
        .times(w.times(p).sinh())
        .div(curr.minus(prev));
  }

  /**
   * Computes Tᵖ for an upper triangular matrix T with nonzero diagonal and p ∈ (−1, 1). See
   * Algorithm 5.1 of [1].
   *
   * @param T Upper triangular matrix.
   * @param p Exponent.
   * @return Tᵖ.
   */
  private static Complex[][] computeTriangularPower(Complex[][] T, double p) {
    int n = T.length;
    var result = zeros(n, n);

    if (n == 1) {
      result[0][0] = T[0][0].pow(p);
      return result;
    } else if (n == 2) {
      compute2x2(T, result, p);
      return result;
    }

    final double maxNormForPade = 2.789358995219730e-1;

    var sqrtT = triu(T);
    Complex[][] IminusT;
    int degree;
    int numberOfSquareRoots = 0;
    boolean hasExtraSquareRoot = false;

    // Take square roots until I − T is small enough for the Padé approximant
    while (true) {
      IminusT = times(sqrtT, -1.0);
      for (int i = 0; i < n; ++i) {
        IminusT[i][i] = IminusT[i][i].plus(1.0);
      }
      double normIminusT = normIndP1(IminusT);
      if (normIminusT < maxNormForPade) {
        degree = getPadeDegree(normIminusT);
        int degree2 = getPadeDegree(normIminusT / 2);
        if (degree - degree2 <= 1 || hasExtraSquareRoot) {
          break;
        }
        hasExtraSquareRoot = true;
      }
      sqrtT = sqrtTriangular(sqrtT);
      ++numberOfSquareRoots;
    }
    result = computePade(degree, IminusT, p);

    // Undo the square roots via repeated squaring
    for (; numberOfSquareRoots > 0; --numberOfSquareRoots) {
      compute2x2(T, result, Math.scalb(p, -numberOfSquareRoots));
      result = times(triu(result), result);
    }
    compute2x2(T, result, p);

    return result;
  }

  /**
   * Computes Aᵖ.
   *
   * @param A Square matrix.
   * @param p Exponent.
   * @return Aᵖ.
   */
  static SimpleMatrix pow(SimpleMatrix A, double p) {
    int n = A.getNumRows();

    if (n == 0) {
      return new SimpleMatrix(0, 0);
    } else if (n == 1) {
      return new SimpleMatrix(1, 1, true, Math.pow(A.get(0, 0), p));
    }

    // Split p into integral part and fractional part in (−1, 1)
    double intpart = Math.floor(p);
    p -= intpart;

    // Perform Schur decomposition only if the power isn't an integer
    ComplexSchur schur = null;
    double conditionNumber = 0.0;
    int rank = n;
    if (p != 0.0) {
      schur = new ComplexSchur(A.getDDRM());

      double maxAbs = 0.0;
      double minAbs = Double.POSITIVE_INFINITY;
      for (int i = 0; i < n; ++i) {
        maxAbs = Math.max(maxAbs, schur.T[i][i].abs());
        minAbs = Math.min(minAbs, schur.T[i][i].abs());
      }
      conditionNumber = maxAbs / minAbs;

      rank = moveZeroEigenvaluesToBottom(schur);
    }

    // Choose the more stable of intpart = floor(p) and intpart = ceil(p)
    if (p > 0.5 && p > (1.0 - p) * Math.pow(conditionNumber, p)) {
      --p;
      ++intpart;
    }

    var result = computeIntPower(A, intpart);
    if (p != 0.0) {
      result = computeFracPower(schur, rank, p).mult(result);
    }
    return result;
  }

  /**
   * Moves zero eigenvalues of the Schur decomposition to the bottom-right corner of T.
   *
   * @param schur Schur decomposition to modify.
   * @return Rank of T.
   */
  private static int moveZeroEigenvaluesToBottom(ComplexSchur schur) {
    var T = schur.T;
    int n = T.length;
    int rank = n;

    for (int i = n - 1; i >= 0; --i) {
      if (rank <= 2) {
        return rank;
      }
      if (T[i][i].isZero()) {
        for (int j = i + 1; j < rank; ++j) {
          var eigenvalue = T[j][j];
          var rot = new Givens(T[j - 1][j], eigenvalue);
          applyOnTheRight(T, j - 1, j, rot, 0, n);
          applyAdjointOnTheLeft(T, j - 1, j, rot, 0, n);
          T[j - 1][j - 1] = eigenvalue;
          T[j][j] = Complex.ZERO;
          applyOnTheRight(schur.U, j - 1, j, rot, 0, n);
        }
        --rank;
      }
    }

    return rank;
  }

  /**
   * Computes Aᵖ for integer p via binary exponentiation.
   *
   * @param A Square matrix.
   * @param p Integer exponent.
   * @return Aᵖ.
   */
  private static SimpleMatrix computeIntPower(SimpleMatrix A, double p) {
    var result = SimpleMatrix.identity(A.getNumRows());
    double pp = Math.abs(p);

    var tmp = p < 0.0 ? A.invert() : A;

    while (true) {
      if (pp % 2.0 >= 1.0) {
        result = tmp.mult(result);
      }
      pp /= 2.0;
      if (pp < 1.0) {
        break;
      }
      tmp = tmp.mult(tmp);
    }

    return result;
  }

  /**
   * Computes Aᵖ = UTᵖUᴴ for p ∈ (−1, 1) from the Schur decomposition of A.
   *
   * @param schur Schur decomposition of A with zero eigenvalues moved to the bottom-right.
   * @param rank Rank of T.
   * @param p Exponent.
   * @return Aᵖ.
   */
  private static SimpleMatrix computeFracPower(ComplexSchur schur, int rank, double p) {
    int n = schur.T.length;
    int nulls = n - rank;

    // Tᵖ for the nonsingular top-left block
    var Ttl = new Complex[rank][rank];
    for (int row = 0; row < rank; ++row) {
      System.arraycopy(schur.T[row], 0, Ttl[row], 0, rank);
    }
    var Ttlp = computeTriangularPower(Ttl, p);

    var Tp = zeros(n, n);
    for (int row = 0; row < rank; ++row) {
      System.arraycopy(Ttlp[row], 0, Tp[row], 0, rank);
    }

    if (nulls > 0) {
      // Top-right block is T_tl⁻¹ T_tlᵖ T_tr
      var Ttr = new Complex[rank][nulls];
      for (int row = 0; row < rank; ++row) {
        System.arraycopy(schur.T[row], rank, Ttr[row], 0, nulls);
      }
      var TpTr = solveUpperTriangular(Ttl, times(triu(Ttlp), Ttr));
      for (int row = 0; row < rank; ++row) {
        System.arraycopy(TpTr[row], 0, Tp[row], rank, nulls);
      }
    }

    // Aᵖ = Re(UTᵖUᴴ)
    var Ap = times(schur.U, times(triu(Tp), adjoint(schur.U)));
    var result = new SimpleMatrix(n, n);
    for (int row = 0; row < n; ++row) {
      for (int col = 0; col < n; ++col) {
        result.set(row, col, Ap[row][col].getReal());
      }
    }
    return result;
  }
}
