/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math;

import edu.wpi.first.wpilibj.math.numbers.N1;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import java.util.Objects;

public final class MatrixUtils {
  private MatrixUtils() {
    throw new AssertionError("utility class");
  }

  public static <R extends Num, C extends Num> Matrix<R, C> zeros(Nat<R> rows, Nat<C> cols) {
    return new Matrix<>(new SimpleMatrix(Objects.requireNonNull(rows).getNum(), Objects.requireNonNull(cols).getNum()));
  }

  public static <N extends Num> Matrix<N, N1> zeros(Nat<N> nums) {
    return new Matrix<>(new SimpleMatrix(Objects.requireNonNull(nums).getNum(), 1));
  }

  public static <D extends Num> Matrix<D, D> eye(Nat<D> dim) {
    return new Matrix<>(SimpleMatrix.identity(Objects.requireNonNull(dim).getNum()));
  }

  public static <R extends Num, C extends Num> Matrix<R, C> ones(Nat<R> rows, Nat<C> cols) {
    final SimpleMatrix temp = new SimpleMatrix(Objects.requireNonNull(rows).getNum(), Objects.requireNonNull(cols).getNum());
    CommonOps_DDRM.fill(temp.getDDRM(), 1.0);
    return new Matrix<>(temp);
  }

  public static <N extends Num> Matrix<N, N1> ones(Nat<N> dim) {
    final SimpleMatrix temp = new SimpleMatrix(Objects.requireNonNull(dim).getNum(), 1);
    CommonOps_DDRM.fill(temp.getDDRM(), 1.0);
    return new Matrix<>(temp);
  }

  public static <R extends Num, C extends Num> MatBuilder<R, C> mat(Nat<R> rows, Nat<C> cols) {
    return new MatBuilder<>(rows, cols);
  }

  public static <D extends Num> VecBuilder<D> vec(Nat<D> dim) {
    return new VecBuilder<>(dim);
  }
}
