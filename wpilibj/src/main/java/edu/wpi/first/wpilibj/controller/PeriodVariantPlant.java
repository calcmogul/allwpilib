/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.*;
import edu.wpi.first.wpilibj.math.numbers.N1;
import org.ejml.UtilEjml;
import org.ejml.dense.block.MatrixMult_DDRB;
import org.ejml.dense.block.MatrixOps_DDRB;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.EigenOps_DDRM;
import org.ejml.dense.row.mult.MatrixMatrixMult_DDRM;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class PeriodVariantPlant<States extends Num, Inputs extends Num, Outputs extends Num> {
  private final double m_nominalSamplePeriod;

  private final Nat<States> kStates;
  private final Nat<Inputs> kInputs;
  private final Nat<Outputs> kOutputs;

  private Matrix<States, N1> m_x;
  private Matrix<Outputs, N1> m_y;
  private Matrix<States, States> m_A;
  private Matrix<States, Inputs> m_B;
  private Matrix<Inputs, N1> m_delayedU;
  private List<PeriodVariantPlantCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;

  public PeriodVariantPlant(PeriodVariantPlantCoeffs<States, Inputs, Outputs> coeffs,
                            double nominalSamplePeriod) {
    m_nominalSamplePeriod = nominalSamplePeriod;
    kStates = coeffs.getStates();
    kInputs = coeffs.getInputs();
    kOutputs = coeffs.getOutputs();

    addCoefficients(coeffs);
    reset();
  }

  public PeriodVariantPlant(PeriodVariantPlantCoeffs<States, Inputs, Outputs> coeffs) {
    this(coeffs, 0.005);
  }

  public Matrix<States, States> getA() {
    return m_A;
  }

  public double getA(int i, int j) {
    return getA().get(i, j);
  }

  public Matrix<States, Inputs> getB() {
    return m_B;
  }

  public double getB(int i, int j) {
    return getB().get(i, j);
  }

  public Matrix<Outputs, States> getC() {
    return getCoefficients().getC();
  }

  public double getC(int i, int j) {
    return getC().get(i, j);
  }

  public Matrix<Outputs, Inputs> getD() {
    return getCoefficients().getD();
  }

  public double getD(int i, int j) {
    return getD().get(i, j);
  }

  public Matrix<States, N1> getX() {
    return m_x;
  }

  public double getX(int i) {
    return getX().get(i, 0);
  }

  public Matrix<Outputs, N1> getY() {
    return m_y;
  }

  public double getY(int i) {
    return getY().get(i, 0);
  }

  public void setX(Matrix<States, N1> x) {
    m_x = x;
  }

  public void setX(int i, double value) {
    m_x.set(i, 0, value);
  }

  public void setY(Matrix<Outputs, N1> y) {
    m_y = y;
  }

  public void setY(int i, double value) {
    m_y.set(i, 0, value);
  }

  public void addCoefficients(PeriodVariantPlantCoeffs<States, Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public PeriodVariantPlantCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }

  public PeriodVariantPlantCoeffs<States, Inputs, Outputs> getCoefficients() {
    return getCoefficients(getIndex());
  }

  public void setIndex(int index) {
    if(index < 0) {
      m_index = 0;
    } else if (index >= m_coefficients.size()) {
      m_index = m_coefficients.size() - 1;
    } else {
      m_index = index;
    }
  }

  public int getIndex() {
    return m_index;
  }

  public void reset() {
    m_x = MatrixUtils.zeros(kStates);
    m_y = MatrixUtils.zeros(kOutputs);
    m_A = MatrixUtils.zeros(kStates, kStates);
    m_B = MatrixUtils.zeros(kStates, kInputs);
    m_delayedU = MatrixUtils.zeros(kInputs);
    updateAB(m_nominalSamplePeriod);
  }

  public void update(Matrix<Inputs, N1> u, double dt) {
    m_x = updateX(getX(), m_delayedU, dt);
    m_y = updateY(m_delayedU);
    m_delayedU = u;
  }

  public Matrix<States, N1> updateX(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dt) {
    updateAB(dt);
    return getA().times(x).plus(getB().times(u));
  }

  public Matrix<Outputs, N1> updateY(Matrix<Inputs, N1> u) {
    return getC().times(getX()).plus(getD().times(u));
  }

  private void updateAB(double dt) {
    SimpleMatrix MstateContinuous = new SimpleMatrix(kStates.getNum() + kInputs.getNum(), kStates.getNum() + kInputs.getNum());

    MstateContinuous.concatColumns(getCoefficients().getAcontinuous().times(dt).getStorage());
    MstateContinuous.concatColumns(getCoefficients().getBcontinuous().times(dt).getStorage());

    SimpleMatrix Mstate = SimpleMatrixUtils.expm(MstateContinuous);
    CommonOps_DDRM.extract(Mstate.getDDRM(), 0, 0, m_A.getStorage().getDDRM());
    CommonOps_DDRM.extract(Mstate.getDDRM(), 0, kStates.getNum(), m_B.getStorage().getDDRM());
  }

  public Nat<States> getStates() {
    return kStates;
  }

  public Nat<Inputs> getInputs() {
    return kInputs;
  }

  public Nat<Outputs> getOutputs() {
    return kOutputs;
  }
}
