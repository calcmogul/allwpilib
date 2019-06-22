/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math;

import org.ejml.simple.SimpleMatrix;

import java.util.Objects;

public class MatBuilder<R extends Num, C extends Num> {
  private final Nat<R> m_rows;
  private final Nat<C> m_cols;

  public final Matrix<R, C> fill(double... data) {
    if (Objects.requireNonNull(data).length != this.m_rows.getNum() * this.m_cols.getNum()) {
      throw new IllegalArgumentException("Invalid matrix data provided. Wanted " + this.m_rows.getNum() + " x " + this.m_cols.getNum() + " matrix, but got " + data.length + " elements");
    } else {
      return new Matrix<>(new SimpleMatrix(this.m_rows.getNum(), this.m_cols.getNum(), true, data));
    }
  }

  public MatBuilder(Nat<R> rows, Nat<C> cols) {
    this.m_rows = Objects.requireNonNull(rows);
    this.m_cols = Objects.requireNonNull(cols);
  }
}
