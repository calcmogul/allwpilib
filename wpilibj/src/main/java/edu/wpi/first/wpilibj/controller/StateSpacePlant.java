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

@SuppressWarnings({"ClassTypeParameterName", "MemberName", "ParameterName"})
public class StateSpacePlant<States extends Num, Inputs extends Num, Outputs extends Num> {
  private List<StateSpacePlantCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
  private int m_index = 0;

  private final Nat<States> kStates;
  private final Nat<Inputs> kInputs;
  private final Nat<Outputs> kOutputs;

  private Matrix<States, N1> m_x;
  private Matrix<Outputs, N1> m_y;

  public StateSpacePlant(StateSpacePlantCoeffs<States, Inputs, Outputs> plantCoeffs) {
    addCoefficients(plantCoeffs);
    kStates = getCoefficients().getStates();
    kInputs = getCoefficients().getInputs();
    kOutputs = getCoefficients().getOutputs();
    reset();
  }

  public Matrix<States, States> getA() {
    return getCoefficients().getA();
  }

  public double getA(int i, int j) {
    return getA().get(i, j);

  }

  public Matrix<States, Inputs> getB() {
    return getCoefficients().getB();
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

  public void addCoefficients(StateSpacePlantCoeffs<States, Inputs, Outputs> coefficients) {
    m_coefficients.add(coefficients);
  }

  public StateSpacePlantCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }

  public StateSpacePlantCoeffs<States, Inputs, Outputs> getCoefficients() {
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

  public void reset() {
    m_x = MatrixUtils.zeros(kStates);
  }

  public void update(Matrix<Inputs, N1> u) {
    m_x = updateX(getX(), u);
    m_y = updateY(getX(), u);
  }

  public Matrix<States, N1> updateX(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
    return getA().times(x).plus(getB().times(u));
  }

  public Matrix<Outputs, N1> updateY(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
    return getC().times(x).plus(getD().times(u));
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
