// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DCMotorInputErrorEstimatorTest {
  @Test
  void testConstantBias() {
    final double kv = 1.0;
    final double ka = 0.1;
    final double dt = 0.005; // s
    final double duration = 2.0; // s

    // Continuous DC motor model
    final double A_c = -kv / ka;
    final double B_c = 1 / ka;

    // Discrete DC motor model
    final double A_d = Math.exp(A_c * dt);
    final double B_d = 1 / A_c * (A_d - 1) * B_c;

    var estimator = new DCMotorInputErrorEstimator(kv, ka, dt);
    double x = 0.0;
    double u = 5.0;
    double u_bias_estimate = 0.0;

    estimator.reset(x);

    // Positive bias
    double u_bias = 2.0;
    for (int i = 0; i < duration / dt; ++i) {
      u_bias_estimate = estimator.calculate(x, u);
      x = A_d * x + B_d * (u + u_bias);
    }
    assertEquals((u + u_bias) / kv, x, 1e-6);
    assertEquals(u_bias, u_bias_estimate, 1e-6);

    // Negative bias
    u_bias = -2.0;
    for (int i = 0; i < duration / dt; ++i) {
      u_bias_estimate = estimator.calculate(x, u);
      x = A_d * x + B_d * (u + u_bias);
    }
    assertEquals((u + u_bias) / kv, x, 1e-6);
    assertEquals(u_bias, u_bias_estimate, 1e-6);

    // Zero bias
    u_bias = 0.0;
    for (int i = 0; i < duration / dt; ++i) {
      u_bias_estimate = estimator.calculate(x, u);
      x = A_d * x + B_d * (u + u_bias);
    }
    assertEquals((u + u_bias) / kv, x, 1e-6);
    assertEquals(u_bias, u_bias_estimate, 1e-6);
  }
}
