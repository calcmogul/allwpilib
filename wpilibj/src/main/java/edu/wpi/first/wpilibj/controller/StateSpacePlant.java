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

  private Nat<States> m_states;
  private Nat<Inputs> m_inputs;
  private Nat<Outputs> m_outputs;

  private Matrix<States, N1> m_x;
  private Matrix<Outputs, N1> m_y;

  public StateSpacePlant(StateSpacePlantCoeffs<States, Inputs, Outputs> plantCoeffs) {
    addCoefficients(plantCoeffs);
    m_states = getCoefficients().getStates();
    m_inputs = getCoefficients().getInputs();
    m_outputs = getCoefficients().getOutputs();
    reset();
  }

  public void update(Matrix<Inputs, N1> u) {
    m_x = updateX(m_x, u);
    m_y = updateY(m_x, u);
  }

  Matrix<States, N1> updateX(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
    // x = Ax + Bu
    return (A().times(x)).plus(B().times(u));
  }

  Matrix<Outputs, N1> updateY(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
    // y = Cx + Du
    return (C().times(x)).plus(D().times(u));
  }

  @SuppressWarnings("MethodName")
  public Matrix<States, States> A() {
    return m_coefficients.get(m_index).getA();
  }

  @SuppressWarnings("MethodName")
  public Matrix<States, Inputs> B() {
    return m_coefficients.get(m_index).getB();
  }

  @SuppressWarnings("MethodName")
  public Matrix<Outputs, States> C() {
    return m_coefficients.get(m_index).getC();
  }

  @SuppressWarnings("MethodName")
  public Matrix<Outputs, Inputs> D() {
    return m_coefficients.get(m_index).getD();
  }

  public Matrix<States, N1> getX() {
    return m_x;
  }

  public double getX(int i) {
    return m_x.get(i, 0);
  }

  public void setX(Matrix<States, N1> x) {
    m_x = x;
  }

  public void setY(Matrix<Outputs, N1> y) {
    m_y = y;
  }

  public Matrix<Outputs, N1> getY() {
    return m_y;
  }

  public double getY(int i) {
    return m_y.get(i, 0);
  }

  public void reset() {
    m_x = MatrixUtils.zeros(m_states);
    m_y = MatrixUtils.zeros(m_outputs);
  }

  public void setIndex(int index) {
    if (index < 0 || index > m_coefficients.size()) {
      throw new IllegalArgumentException("Invalid index for state space coefficients list of size " + m_coefficients.size());
    }
    m_index = index;
  }

  public int getIndex() {
    return m_index;
  }

  private void addCoefficients(StateSpacePlantCoeffs<States, Inputs, Outputs> plantCoeffs) {
    m_coefficients.add(plantCoeffs);
  }

  public Nat<States> getStates() {
    return m_states;
  }

  public Nat<Inputs> getInputs() {
    return m_inputs;
  }

  public Nat<Outputs> getOutputs() {
    return m_outputs;
  }

  public StateSpacePlantCoeffs<States, Inputs, Outputs> getCoefficients() {
    return m_coefficients.get(m_index);
  }

  public StateSpacePlantCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
    return m_coefficients.get(index);
  }
}
