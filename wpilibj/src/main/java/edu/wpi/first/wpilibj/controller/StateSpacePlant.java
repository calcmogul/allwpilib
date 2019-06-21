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

public class StateSpacePlant<States extends Num, Inputs extends Num, Outputs extends Num> {
    private List<StateSpacePlantCoeffs<States, Inputs, Outputs>> m_coefficients = new ArrayList<>();
    private int m_index = 0;

    private Nat<States> m_states;
    private Nat<Inputs> m_inputs;
    private Nat<Outputs> m_outputs;

    private Matrix<States, N1> m_X;
    private Matrix<Outputs, N1> m_Y;

    public StateSpacePlant(StateSpacePlantCoeffs<States, Inputs, Outputs> plantCoeffs) {
        addCoefficients(plantCoeffs);
        m_states = getCoefficients().getStates();
        m_inputs = getCoefficients().getInputs();
        m_outputs = getCoefficients().getOutputs();
        reset();
    }

    public void update(Matrix<Inputs, N1> u) {
        m_X = updateX(m_X, u);
        m_Y = updateY(m_X, u);
    }

    Matrix<States, N1> updateX(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
        // x = Ax + Bu
        return (A().times(x)).plus(B().times(u));
    }

    Matrix<Outputs, N1> updateY(Matrix<States, N1> x, Matrix<Inputs, N1> u) {
        // y = Cx + Du
        return (C().times(x)).plus(D().times(u));
    }

    public Matrix<States, States> A() {
        return m_coefficients.get(m_index).getA();
    }

    public Matrix<States, Inputs> B() {
        return m_coefficients.get(m_index).getB();
    }

    public Matrix<Outputs, States> C() {
        return m_coefficients.get(m_index).getC();
    }

    public Matrix<Outputs, Inputs> D() {
        return m_coefficients.get(m_index).getD();
    }

    public Matrix<States, N1> getX() {
        return m_X;
    }

    public double getX(int i) {
        return m_X.get(i, 0);
    }

    public void setX(Matrix<States, N1> x) {
        m_X = x;
    }

    public void setY(Matrix<Outputs, N1> y) {
        m_Y = y;
    }

    public Matrix<Outputs, N1> getY() {
        return m_Y;
    }

    public double getY(int i) {
        return m_Y.get(i, 0);
    }

    public void reset() {
        m_X = MatrixUtils.zeros(m_states);
        m_Y = MatrixUtils.zeros(m_outputs);
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
