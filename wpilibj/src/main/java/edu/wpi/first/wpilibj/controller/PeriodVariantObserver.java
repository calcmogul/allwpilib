/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.MatrixUtils;
import edu.wpi.first.wpilibj.math.Num;
import edu.wpi.first.wpilibj.math.SimpleMatrixUtils;
import edu.wpi.first.wpilibj.math.numbers.N1;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class PeriodVariantObserver<States extends Num, Inputs extends Num, Outputs extends Num> {
  private PeriodVariantPlant<States, Inputs, Outputs> m_plant;
  private double m_nominalSamplePeriod;
  private Matrix<States, N1> m_Xhat;
  private Matrix<States, States> m_P;
  private Matrix<States, States> m_Q;
  private Matrix<Outputs, Outputs> m_R;
  private List<PeriodVariantObserverCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;

  private void updateQR(double dt) {
    int states = m_plant.getStates().getNum();
    SimpleMatrix Qtemp = (getCoefficients().getQcontinuous().plus(getCoefficients().getQcontinuous().transpose()))
        .div(2.0).getStorage();
    SimpleMatrix Rtemp = (getCoefficients().getRcontinuous().plus(getCoefficients().getRcontinuous().transpose()))
        .div(2.0).getStorage();

    Qtemp.concatRows(m_plant.getCoefficients().getAcontinuous().getStorage().transpose());

    SimpleMatrix Mgain = new SimpleMatrix(states * 2, states * 2);
    Mgain.concatColumns(m_plant.getCoefficients().getAcontinuous().getStorage().negative(), Qtemp);

    SimpleMatrix phi = SimpleMatrixUtils.expm(Mgain.scale(dt));
    SimpleMatrix phi12 = new SimpleMatrix(states, states);
    CommonOps_DDRM.extract(phi.getDDRM(), 0, states, phi12.getDDRM());
    SimpleMatrix phi22 = new SimpleMatrix(states, states);
    CommonOps_DDRM.extract(phi.getDDRM(), states, states, phi22.getDDRM());

    m_Q = new Matrix<>(phi22.transpose().mult(phi12));
    m_Q = (m_Q.plus(m_Q.transpose())).div(2.0);
    m_R = new Matrix<>(Rtemp.divide(dt));
  }

  public PeriodVariantObserver(PeriodVariantObserverCoeffs<States, Inputs, Outputs> coeffs,
                               PeriodVariantPlant<States, Inputs, Outputs> plant,
                               double nominalSamplePeriod) {
    addCoefficients(coeffs);
    m_plant = plant;
    m_nominalSamplePeriod = nominalSamplePeriod;
  }

  public PeriodVariantObserver(PeriodVariantObserverCoeffs<States, Inputs, Outputs> coeffs,
                               PeriodVariantPlant<States, Inputs, Outputs> plant) {
    this(coeffs, plant, 0.005);
  }

  public Matrix<States, States> getQ() {
    return m_Q;
  }

  public double getQ(int i, int j) {
    return getQ().get(i, j);
  }

  public Matrix<Outputs, Outputs> getR() {
    return m_R;
  }

  public double getR(int i, int j) {
    return getR().get(i, j);
  }

  public Matrix<States, States> getP() {
    return m_P;
  }

  public double getP(int i, int j) {
    return getP().get(i, j);
  }

  public Matrix<States, N1> getXhat() {
    return m_Xhat;
  }

  public double getXhat(int i) {
    return getXhat().get(i, 0);
  }

  public void setXhat(Matrix<States, N1> xHat) {
    m_Xhat = xHat;
  }

  public void setXhat(int i, double value) {
    m_Xhat.set(i, 0, value);
  }

  public void reset() {
    m_Xhat = MatrixUtils.zeros(m_plant.getStates());
    m_P = getCoefficients().getPsteadyState();
    updateQR(m_nominalSamplePeriod);
  }

  public void predict(Matrix<Inputs, N1> newU, double dt) {
    m_Xhat = m_plant.updateX(getXhat(), newU, dt);

    updateQR(dt);

    m_P = m_plant.getA().times(getP()).times(m_plant.getA().transpose()).plus(m_Q);
  }

  public void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> y) {
    Matrix<Outputs, N1> yBar = y.minus(m_plant.getC().times(getXhat()).plus(m_plant.getD().times(u)));
    Matrix<Outputs, Outputs> S = m_plant.getC().times(getP()).times(m_plant.getC().transpose()).plus(getR());
    Matrix<States, Outputs> kalmanGain = getP().times(m_plant.getC().transpose()).times(S.inv());

    m_Xhat = getXhat().plus(kalmanGain.times(yBar));
    m_P = (MatrixUtils.eye(m_plant.getStates()).minus(kalmanGain.times(m_plant.getC()))).times(getP());
  }

  public void addCoefficients(PeriodVariantObserverCoeffs<States, Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public PeriodVariantObserverCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }

  public PeriodVariantObserverCoeffs<States, Inputs, Outputs> getCoefficients() {
    return getCoefficients(m_index);
  }

  public void setIndex(int index) {
    if(index < 0) {
      m_index = 0;
    } else if(index >= m_coefficients.size()) {
      m_index = m_coefficients.size() - 1;
    } else {
      m_index = index;
    }
  }

  public int getIndex() {
    return m_index;
  }
}
