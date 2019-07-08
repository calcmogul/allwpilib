/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.MatrixUtils;
import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;
import edu.wpi.first.wpilibj.math.numbers.N1;

import java.util.ArrayList;
import java.util.List;

public class OutputController<Inputs extends Num, Outputs extends Num> {
  private boolean m_enabled = false;
  private Matrix<Outputs, N1> m_r;
  private Matrix<Inputs, N1> m_u;
  private List<OutputControllerCoeffs<Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;

  private final Nat<Outputs> kOutputs;
  private final Nat<Inputs> kInputs;

  public OutputController(OutputControllerCoeffs<Inputs, Outputs> coefficients, Nat<Outputs> outputs, Nat<Inputs> inputs) {
    addCoefficients(coefficients);
    reset();
    kOutputs = outputs;
    kInputs = inputs;
  }

  public void enable() {
    m_enabled = true;
  }

  public void disable() {
    m_enabled = false;
  }

  public Matrix<Inputs, Outputs> getK() {
    return getCoefficients().getK();
  }

  public double getK(int i, int j) {
    return getK().get(i, j);
  }

  public Matrix<Outputs, N1> getR() {
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
    m_r = MatrixUtils.zeros(kOutputs);
    m_u = MatrixUtils.zeros(kInputs);
  }

  public void update(Matrix<Outputs, N1> y) {
    if(m_enabled) {
      m_u = getK().times(m_r.minus(y));
      capU();
    }
  }

  public void update(Matrix<Outputs, N1> y, Matrix<Outputs, N1> nextR) {
    if(m_enabled) {
      m_u = getK().times(m_r.minus(y));
      capU();
      m_r = nextR;
    }
  }

  private void capU() {
    for(int i = 0; i < kInputs.getNum(); i++) {
      if(getU(i) > getCoefficients().getUmax(i)) {
        m_u.set(i, 0, getCoefficients().getUmax(i));
      }else if(getU(i) < getCoefficients().getUmin(i)) {
        m_u.set(i, 0, getCoefficients().getUmin(i));
      }
    }
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

  public void addCoefficients(OutputControllerCoeffs<Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public OutputControllerCoeffs<Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }


  public OutputControllerCoeffs<Inputs, Outputs> getCoefficients() {
    return getCoefficients(m_index);
  }
}
