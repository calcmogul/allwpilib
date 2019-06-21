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

    public void setIndex(int index) {
        if (index < 0 || index > m_coefficients.size()) {
            throw new IllegalArgumentException("Invalid coefficients index for observer with coefficients size " + m_coefficients.size());
        }
        m_index = index;
    }

    public Matrix<States, Outputs> K() {
        return m_coefficients.get(m_index).getK();
    }

    public Matrix<States, N1> getXhat() {
        return m_Xhat;
    }

    public void setXhat(Matrix<States, N1> Xhat) {
        m_Xhat = Xhat;
    }

    public void predict(Matrix<Inputs, N1> newU) {
        m_Xhat = m_plant.updateX(m_Xhat, newU);
    }

    public void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> y) {
        // Xhat += K * (y - Cx - Du)
        m_Xhat = m_Xhat.plus(K().times(y.minus(m_plant.updateY(m_Xhat, u))));
    }

    public void reset() {
        m_Xhat = MatrixUtils.zeros(m_plant.getStates());
    }

    public StateSpaceObserverCoeffs<States, Inputs, Outputs> getCoefficients() {
        return m_coefficients.get(m_index);
    }

    public StateSpaceObserverCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
        return m_coefficients.get(index);
    }

    public void addCoefficients(StateSpaceObserverCoeffs<States, Inputs, Outputs> coeffs) {
        m_coefficients.add(coeffs);
    }
}
