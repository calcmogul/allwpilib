// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.numbers.N5;
import org.wpilib.math.numbers.N9;
import org.wpilib.math.util.Nat;

class DiscretizationTest {
  // Check that for a simple second-order system that we can easily analyze
  // analytically,
  @Test
  void testDiscretizeA() {
    final var contA = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, 0);
    final var x0 = VecBuilder.fill(1, 1);

    final var discA = Discretization.discretizeA(contA, 1.0);
    final var x1Discrete = discA.times(x0);

    // We now have pos = vel = 1 and accel = 0, which should give us:
    final var x1Truth =
        VecBuilder.fill(
            1.0 * x0.get(0, 0) + 1.0 * x0.get(1, 0), 0.0 * x0.get(0, 0) + 1.0 * x0.get(1, 0));

    assertEquals(x1Truth, x1Discrete);
  }

  // Check that for a simple second-order system that we can easily analyze
  // analytically,
  @Test
  void testDiscretizeAB() {
    final var contA = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, 0);
    final var contB = MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 1);

    final var x0 = VecBuilder.fill(1, 1);
    final var u = VecBuilder.fill(1);

    var discABPair = Discretization.discretizeAB(contA, contB, 1.0);
    var discA = discABPair.getFirst();
    var discB = discABPair.getSecond();

    var x1Discrete = discA.times(x0).plus(discB.times(u));

    // We now have pos = vel = accel = 1, which should give us:
    final var x1Truth =
        VecBuilder.fill(
            1.0 * x0.get(0, 0) + 1.0 * x0.get(1, 0) + 0.5 * u.get(0, 0),
            0.0 * x0.get(0, 0) + 1.0 * x0.get(1, 0) + 1.0 * u.get(0, 0));

    assertEquals(x1Truth, x1Discrete);
  }

  // Test that discretizeAB() matches the dense matrix exponential for a system large enough to use
  // the block triangular matrix exponential
  @Test
  void testDiscretizeABMatchesDenseExp() {
    final var rand = new Random(0);
    final var contA = new Matrix<N5, N5>(SimpleMatrix.random_DDRM(5, 5, -1.0, 1.0, rand));
    final var contB = new Matrix<N5, N2>(SimpleMatrix.random_DDRM(5, 2, -1.0, 1.0, rand));

    for (double dt : new double[] {1e-4, 5e-3, 0.1, 2.0, 10.0}) {
      final var discABPair = Discretization.discretizeAB(contA, contB, dt);

      // M = [A  B]
      //     [0  0]
      var M = new Matrix<>(new SimpleMatrix(7, 7));
      M.assignBlock(0, 0, contA);
      M.assignBlock(0, 5, contB);
      var phi = M.times(dt).exp();
      Matrix<N5, N5> expectedA = phi.block(5, 5, 0, 0);
      Matrix<N5, N2> expectedB = phi.block(5, 2, 0, 5);

      assertTrue(isApprox(discABPair.getFirst(), expectedA, 1e-12));
      assertTrue(isApprox(discABPair.getSecond(), expectedB, 1e-12));
    }
  }

  // Test that discretizeAQ() matches the dense matrix exponential for a system large enough to use
  // the block triangular matrix exponential
  @Test
  void testDiscretizeAQMatchesDenseExp() {
    final var rand = new Random(0);
    final var contA = new Matrix<N9, N9>(SimpleMatrix.random_DDRM(9, 9, -1.0, 1.0, rand));
    final var L = new Matrix<N9, N9>(SimpleMatrix.random_DDRM(9, 9, -1.0, 1.0, rand));
    final var contQ = L.times(L.transpose());

    for (double dt : new double[] {1e-4, 5e-3, 0.1, 0.5}) {
      final var discAQPair = Discretization.discretizeAQ(contA, contQ, dt);

      // M = [−A  Q ]
      //     [ 0  Aᵀ]
      var M = new Matrix<>(new SimpleMatrix(18, 18));
      M.assignBlock(0, 0, contA.times(-1.0));
      M.assignBlock(0, 9, contQ);
      M.assignBlock(9, 9, contA.transpose());
      var phi = M.times(dt).exp();
      Matrix<N9, N9> phi12 = phi.block(9, 9, 0, 9);
      Matrix<N9, N9> phi22 = phi.block(9, 9, 9, 9);
      var expectedA = phi22.transpose();
      var expectedQ = expectedA.times(phi12);
      expectedQ = expectedQ.plus(expectedQ.transpose()).div(2.0);

      assertTrue(isApprox(discAQPair.getFirst(), expectedA, 1e-12));
      assertTrue(isApprox(discAQPair.getSecond(), expectedQ, 1e-12));
    }
  }

  private static boolean isApprox(Matrix<?, ?> actual, Matrix<?, ?> expected, double tolerance) {
    return actual.getStorage().minus(expected.getStorage()).normF() <= tolerance * expected.normF();
  }

  //                                               T
  // Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //                                               0
  @Test
  void testDiscretizeSlowModelAQ() {
    final var contA = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, 0);
    final var contQ = MatBuilder.fill(Nat.N2(), Nat.N2(), 1, 0, 0, 1);

    final double dt = 1.0;

    //       T
    // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
    //       0
    final var discQIntegrated =
        NumericalIntegration.rk4(
            (Double t, Matrix<N2, N2> x) ->
                contA.times(t).exp().times(contQ).times(contA.transpose().times(t).exp()),
            0.0,
            new Matrix<>(Nat.N2(), Nat.N2()),
            dt);

    var discAQPair = Discretization.discretizeAQ(contA, contQ, dt);
    var discQ = discAQPair.getSecond();

    assertTrue(
        discQIntegrated.minus(discQ).normF() < 1e-10,
        "Expected these to be nearly equal:\ndiscQ:\n"
            + discQ
            + "\ndiscQIntegrated:\n"
            + discQIntegrated);
  }

  //                                               T
  // Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //                                               0
  @Test
  void testDiscretizeFastModelAQ() {
    final var contA = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, -1406.29);
    final var contQ = MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0025, 0, 0, 1);

    final var dt = 0.005;

    //       T
    // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
    //       0
    final var discQIntegrated =
        NumericalIntegration.rk4(
            (Double t, Matrix<N2, N2> x) ->
                contA.times(t).exp().times(contQ).times(contA.transpose().times(t).exp()),
            0.0,
            new Matrix<>(Nat.N2(), Nat.N2()),
            dt);

    var discAQPair = Discretization.discretizeAQ(contA, contQ, dt);
    var discQ = discAQPair.getSecond();

    assertTrue(
        discQIntegrated.minus(discQ).normF() < 1e-3,
        "Expected these to be nearly equal:\ndiscQ:\n"
            + discQ
            + "\ndiscQIntegrated:\n"
            + discQIntegrated);
  }

  // Test that DiscretizeR() works
  @Test
  void testDiscretizeR() {
    var contR = MatBuilder.fill(Nat.N2(), Nat.N2(), 2.0, 0.0, 0.0, 1.0);
    var discRTruth = MatBuilder.fill(Nat.N2(), Nat.N2(), 4.0, 0.0, 0.0, 2.0);

    var discR = Discretization.discretizeR(contR, 0.5);

    assertTrue(
        discRTruth.minus(discR).normF() < 1e-10,
        "Expected these to be nearly equal:\ndiscR:\n" + discR + "\ndiscRTruth:\n" + discRTruth);
  }
}
