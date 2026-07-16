// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.estimator;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.util.Nat;

/**
 * This class estimates the input error required to make the expected motor velocity match the
 * actual velocity over one timestep.
 *
 * <p>Subtracting this input error from the next control input will correct for the bias and
 * eliminate steady-state error. This method of fixing steady-state error is superior to a pure
 * integral controller because it only integrates for unmodeled disturbances, not during expected
 * transient behavior while converging to a reference. This means there won't be overshoot during
 * transients.
 */
public class DCMotorInputErrorEstimator {
  private final double m_dt;
  private final SteadyStateKalmanFilter<N2, N1, N1> m_kf;

  /**
   * Constructs a DC motor input error estimator.
   *
   * @param Kv The velocity feedforward gain.
   * @param Ka The acceleration feedforward gain.
   * @param dt Nominal discretization timestep in seconds.
   */
  public DCMotorInputErrorEstimator(double Kv, double Ka, double dt) {
    m_dt = dt;
    m_kf =
        new SteadyStateKalmanFilter<N2, N1, N1>(
            Nat.N2(),
            Nat.N1(),
            new LinearSystem<N2, N1, N1>(
                new Matrix<>(Nat.N2(), Nat.N2(), new double[] {-Kv / Ka, 1 / Ka, 0, 0}),
                new Matrix<>(Nat.N2(), Nat.N1(), new double[] {1 / Ka, 0}),
                new Matrix<>(Nat.N1(), Nat.N2(), new double[] {1, 0}),
                new Matrix<>(Nat.N1(), Nat.N1(), new double[] {0})),
            // TODO: Find velocity model standard deviation and velocity measurement
            // standard deviation dynamically
            VecBuilder.fill(0.1, 12.0),
            VecBuilder.fill(0.1),
            m_dt);
  }

  /**
   * Returns the input error estimate.
   *
   * @param velocity The velocity from the current timestep.
   * @param voltage The voltage applied for the current timestep.
   * @return The input error estimate.
   */
  public double calculate(double velocity, double voltage) {
    // NOTE: correct() should be called with the voltage from the previous
    // timestep instead of the current timestep, but it doesn't affect accuracy
    // because the system's D matrix is zero.
    m_kf.correct(VecBuilder.fill(voltage), VecBuilder.fill(velocity));
    m_kf.predict(VecBuilder.fill(voltage), m_dt);

    return m_kf.getXhat(1);
  }

  /**
   * Resets the estimator to the given velocity with no input error.
   *
   * @param velocity The current velocity.
   */
  public void reset(double velocity) {
    m_kf.setXhat(VecBuilder.fill(velocity, 0.0));
  }
}
