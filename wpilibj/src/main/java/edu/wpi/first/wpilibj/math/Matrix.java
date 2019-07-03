/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math;

import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import java.util.Objects;

/**
 * A shape-safe wrapper over Efficient Java Matrix Library (EJML) matrices
 * <p>
 * This class is intended to be used alongside the state space library
 *
 * @param <R> The number of rows in this matrix
 * @param <C> The number of columns in this matrix
 */
public class Matrix<R extends Num, C extends Num> {

  private final SimpleMatrix m_storage;

  /**
   * Gets the number of columns in this matrix
   *
   * @return The number of columns, according to the internal storage
   */
  public final int getNumCols() {
    return this.m_storage.numCols();
  }

  /**
   * Gets the number of rows in this matrix
   *
   * @return The number of rows, according to the internal storage
   */
  public final int getNumRows() {
    return this.m_storage.numRows();
  }

  /**
   * Get an element of this matrix
   *
   * @param i The row of the element
   * @param j The column of the element
   * @return The element in this matrix at ij
   */
  @SuppressWarnings("ParameterName")
  public final double get(int i, int j) {
    return this.m_storage.get(i, j);
  }

  /**
   * Sets the value at the given indices
   *
   * @param i     The row of the element
   * @param j     The column of the element
   * @param value The value to insert at the given location
   */
  @SuppressWarnings("ParameterName")
  public final void set(int i, int j, double value) {
    this.m_storage.set(i, j, value);
  }


  public final Matrix<R, C> diag() {
    return new Matrix<>(this.m_storage.diag());
  }

  /**
   * Returns the largest element of this matrix
   *
   * @return The largest element of this matrix
   */
  public final double maxInternal() {
    return CommonOps_DDRM.elementMax(this.m_storage.getDDRM());
  }

  /**
   * Returns the smallest element of this matrix
   *
   * @return The smallest element of this matrix
   */
  public final double minInternal() {
    return CommonOps_DDRM.elementMin(this.m_storage.getDDRM());
  }

  /**
   * Calculates the mean of the elements in this matrix
   *
   * @return The mean value of this matrix
   */
  public final double mean() {
    return this.elementSum() / (double) this.m_storage.getNumElements();
  }

  /**
   * Multiplies this matrix with another that has <C> rows.
   * As matrix multiplication is only defined if the number of columns in the first matrix matches the number of rows in the second,
   * this operation will fail to compile under any other circumstances
   *
   * @param other The other matrix to multiply by
   * @param <C2> The number of columns in the second matrix
   * @return The result of the matrix multiplication between this and the given matrix
   */
  public final <C2 extends Num> Matrix<R, C2> times(Matrix<C, C2> other) {
    return new Matrix<>(this.m_storage.mult(other.m_storage));
  }

  /**
   * Multiplies all the elements of this matrix by the given scalar
   *
   * @param value The scalar value to multiply by
   * @return A new matrix with all the elements multiplied by the given value
   */
  public final Matrix<R, C> times(double value) {
    return new Matrix<>(this.m_storage.scale(value));
  }


  public final Matrix<R, C> elementTimes(Matrix<R, C> other) {
    return new Matrix<>(this.m_storage.elementMult(Objects.requireNonNull(other).m_storage));
  }

  /**
   * Subtracts the given value from all the elements of this matrix
   *
   * @param value The value to subtract
   * @return The resultant matrix
   */
  public final Matrix<R, C> minus(double value) {
    return new Matrix<>(this.m_storage.minus(value));
  }


  public final Matrix<R, C> minus(Matrix<R, C> value) {
    return new Matrix<>(this.m_storage.minus(Objects.requireNonNull(value).m_storage));
  }


  public final Matrix<R, C> plus(double value) {
    return new Matrix<>(this.m_storage.plus(value));
  }


  public final Matrix<R, C> plus(Matrix<R, C> value) {
    return new Matrix<>(this.m_storage.plus(value.m_storage));
  }


  public final Matrix<R, C> div(int value) {
    return new Matrix<>(this.m_storage.divide((double) value));
  }


  public final Matrix<R, C> div(double value) {
    return new Matrix<>(this.m_storage.divide(value));
  }


  public final Matrix<C, R> transpose() {
    return new Matrix<>(this.m_storage.transpose());
  }


  public final Matrix<R, C> copy() {
    return new Matrix<>(this.m_storage.copy());
  }


  public final Matrix<R, C> inv() {
    return new Matrix<>(this.m_storage.invert());
  }

  public final double det() {
    return this.m_storage.determinant();
  }

  public final double normF() {
    return this.m_storage.normF();
  }

  public final double normIndP1() {
    return NormOps_DDRM.inducedP1(this.m_storage.getDDRM());
  }

  public final double elementSum() {
    return this.m_storage.elementSum();
  }

  public final double trace() {
    return this.m_storage.trace();
  }


  public final Matrix<R, C> epow(double other) {
    return new Matrix<>(this.m_storage.elementPower(other));
  }


  public final Matrix<R, C> epow(int other) {
    return new Matrix<>(this.m_storage.elementPower((double) other));
  }


  final SimpleMatrix getStorage() {
    return this.m_storage;
  }

  Matrix(SimpleMatrix storage) {
    this.m_storage = Objects.requireNonNull(storage);
  }
}
