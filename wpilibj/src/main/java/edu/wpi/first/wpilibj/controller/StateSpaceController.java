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
        m_u = MatrixUtils.zeros(m_plant.getInputs());
    }

    public void reset() {
        m_r = MatrixUtils.zeros(m_plant.getStates());
        m_u = MatrixUtils.zeros(m_plant.getInputs());
    }

    public void update(Matrix<States, N1> x) {
        if (m_enabled) {
            m_u = (K().times(m_r.minus(x))).plus(Kff().times(m_r.minus(m_plant.A().times(m_r))));
        }
    }

    public void update(Matrix<States, N1> nextR, Matrix<States, N1> x) {
        if (m_enabled) {
            // u = K * (r - x) + Kff * (nextR - Ar)
            m_u = (K().times(m_r.minus(x))).plus(Kff().times(nextR.minus(m_plant.A().times(m_r))));

            m_r = nextR;
        }
    }

    private void capU() {
        for (int i = 0; i < m_plant.getInputs().getNum(); i++) {
            if (U(i) > getCoefficients().getUmax().get(i, 0)) {
                m_u.set(i, 0, getCoefficients().getUmax().get(i, 0));
            }else if(U(i) < getCoefficients().getUmin().get(i, 0)) {
                m_u.set(i, 0, getCoefficients().getUmin().get(i, 0));
            }
        }
    }

    public void addCoefficients(StateSpaceControllerCoeffs<States, Inputs, Outputs> coeffs) {
        m_coefficients.add(coeffs);
    }

    public StateSpaceControllerCoeffs<States, Inputs, Outputs> getCoefficients() {
        return m_coefficients.get(m_index);
    }

    public StateSpaceControllerCoeffs<States, Inputs, Outputs> getCoefficients(int index) {
        return m_coefficients.get(index);
    }

    public int getIndex() {
        return m_index;
    }

    public void setIndex(int index) {
        if(index < 0 || index > m_coefficients.size()) {
            throw new IllegalArgumentException("Invalid index given for state space controller with " + m_coefficients.size() + "  coefficients");
        }
        m_index = index;
    }

    public Matrix<Inputs, States> K() {
        return m_coefficients.get(m_index).getK();
    }

    public Matrix<Inputs, States> Kff() {
        return m_coefficients.get(m_index).getKff();
    }

    public Matrix<Inputs, N1> U() {
        return m_u;
    }

    public double U(int i) {
        return m_u.get(i, 0);
    }
}
