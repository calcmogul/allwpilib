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

public class Matrix<R extends Num, C extends Num> {

  private final SimpleMatrix m_storage;

  public final int getNumCols() {
    return this.m_storage.numCols();
  }

  public final int getNumRows() {
    return this.m_storage.numRows();
  }

  @SuppressWarnings("ParameterName")
  public final double get(int i, int j) {
    return this.m_storage.get(i, j);
  }

  @SuppressWarnings("ParameterName")
  public final void set(int i, int j, double k) {
    this.m_storage.set(i, j, k);
  }


  public final Matrix<R, C> diag() {
    return new Matrix<>(this.m_storage.diag());
  }

  public final double maxInternal() {
    return CommonOps_DDRM.elementMax(this.m_storage.getDDRM());
  }

  public final double minInternal() {
    return CommonOps_DDRM.elementMin(this.m_storage.getDDRM());
  }

  public final double mean() {
    return this.elementSum() / (double) this.m_storage.getNumElements();
  }


  public final <C2 extends Num> Matrix<R, C2> times(Matrix<C, C2> other) {
    return new Matrix<>(this.m_storage.mult(other.m_storage));
  }


  public final Matrix<R, C> times(double value) {
    return new Matrix<>(this.m_storage.scale(value));
  }


  public final Matrix<R, C> elementTimes(Matrix<R, C> other) {
    return new Matrix<>(this.m_storage.elementMult(Objects.requireNonNull(other).m_storage));
  }


  public final Matrix<R, C> unaryMinus() {
    return new Matrix<>(this.m_storage.scale(-1.0D));
  }


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
