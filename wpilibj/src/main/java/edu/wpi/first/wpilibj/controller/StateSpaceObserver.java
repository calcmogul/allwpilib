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
import edu.wpi.first.wpilibj.math.numbers.N1;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"ClassTypeParameterName", "MemberName", "ParameterName"})
public class StateSpaceObserver<States extends Num, Inputs extends Num, Outputs extends Num> {
  private StateSpacePlant<States, Inputs, Outputs> m_plant;
  private List<StateSpaceObserverCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;

  private Matrix<States, N1> m_Xhat;

  public StateSpaceObserver(StateSpacePlant<States, Inputs, Outputs> plant, StateSpaceObserverCoeffs<States, Inputs, Outputs> coeffs) {
    m_plant = plant;
    addCoefficients(coeffs);
    reset();
  }

  public Matrix<States, Outputs> getK() {
    return getCoefficients().getK();
  }

  public double getK(int i, int j) {
    return getK().get(i, j);
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
  }

  public void predict(Matrix<Inputs, N1> newU) {
    m_Xhat = m_plant.updateX(getXhat(), newU);
  }

  public void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> y) {
    m_Xhat = m_Xhat.plus(getK().times(y.minus(m_plant.updateY(getXhat(), u))));
  }

  public void addCoefficients(StateSpaceObserverCoeffs<States, Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public StateSpaceObserverCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }

  public StateSpaceObserverCoeffs<States, Inputs, Outputs> getCoefficients() {
    return getCoefficients(getIndex());
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
