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
public class StateSpaceController<States extends Num, Inputs extends Num, Outputs extends Num> {
  private StateSpacePlant<States, Inputs, Outputs> m_plant;
  private List<StateSpaceControllerCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;
  private boolean m_enabled = false;

  private Matrix<States, N1> m_r;
  private Matrix<Inputs, N1> m_u;

  public StateSpaceController(StateSpacePlant<States, Inputs, Outputs> plant, StateSpaceControllerCoeffs<States, Inputs, Outputs> coeffs) {
    m_plant = plant;
    addCoefficients(coeffs);
    reset();
  }

  public void enable() {
    m_enabled = true;
  }

  public void disable() {
    m_enabled = false;
  }

  public Matrix<Inputs, States> getK() {
    return getCoefficients().getK();
  }

  public double getK(int i, int j) {
    return getK().get(i, j);
  }

  public Matrix<Inputs, States> getKff() {
    return getCoefficients().getKff();
  }

  public double getKff(int i, int j) {
    return getKff().get(i, j);
  }

  public Matrix<States, N1> getR() {
    return m_r;
  }

  public double getR(int i) {
    return getR().get(i, 0);
  }

  public Matrix<Inputs, N1> getU() {
    return m_u;
  }

  public double getU(int i) {
    return getU().get(i, 0);
  }

  public void reset() {
    m_r = MatrixUtils.zeros(m_plant.getStates());
    m_u = MatrixUtils.zeros(m_plant.getInputs());
  }

  public void update(Matrix<States, N1> x) {
    if (m_enabled) {
      m_u = getK().times(m_r.minus(x)).plus(getKff().times(m_r.minus(m_plant.getA().times(m_r))));
      capU();
    }
  }

  public void update(Matrix<States, N1> nextR, Matrix<States, N1> x) {
    if (m_enabled) {
      m_u = getK().times(m_r.minus(x)).plus(getKff().times(nextR.minus(m_plant.getA().times(m_r))));
      capU();
      m_r = nextR;
    }
  }

  private void capU() {
    for(int i = 0; i < m_plant.getInputs().getNum(); ++i) {
      if(getU(i) > getCoefficients().getUmax(i)) {
        m_u.set(i, 0, getCoefficients().getUmax(i));
      } else if(getU(i) < getCoefficients().getUmin(i)) {
        m_u.set(i, 0, getCoefficients().getUmin(i));
      }
    }
  }

  public void setIndex(int index) {
    if(index < 0) {
      m_index = 0;
    }else if(index >= m_coefficients.size()) {
      m_index = m_coefficients.size() - 1;
    }else {
      m_index = index;
    }
  }

  public int getIndex() {
    return m_index;
  }

  public void addCoefficients(StateSpaceControllerCoeffs<States, Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public StateSpaceControllerCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }

  public StateSpaceControllerCoeffs<States, Inputs, Outputs> getCoefficients() {
    return getCoefficients(getIndex());
  }
}
