// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.system;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import org.ejml.simple.SimpleMatrix;

/** Discretization helper functions. */
public final class Discretization {
  private Discretization() {
    // Utility class
  }

  /**
   * Discretizes the given continuous A matrix.
   *
   * @param <States> Num representing the number of states.
   * @param contA Continuous system matrix.
   * @param dtSeconds Discretization timestep.
   * @return the discrete matrix system.
   */
  public static <States extends Num> Matrix<States, States> discretizeA(
      Matrix<States, States> contA, double dtSeconds) {
    // A_d = eᴬᵀ
    return contA.times(dtSeconds).exp();
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param <States> Nat representing the states of the system.
   * @param <Inputs> Nat representing the inputs to the system.
   * @param contA Continuous system matrix.
   * @param contB Continuous input matrix.
   * @param dtSeconds Discretization timestep.
   * @return a Pair representing discA and diskB.
   */
  public static <States extends Num, Inputs extends Num>
      Pair<Matrix<States, States>, Matrix<States, Inputs>> discretizeAB(
          Matrix<States, States> contA, Matrix<States, Inputs> contB, double dtSeconds) {
    int states = contA.getNumRows();
    int inputs = contB.getNumCols();

    // M = [A  B]
    //     [0  0]
    var M = new Matrix<>(new SimpleMatrix(states + inputs, states + inputs));
    M.assignBlock(0, 0, contA);
    M.assignBlock(0, contA.getNumCols(), contB);

    //  ϕ = eᴹᵀ = [A_d  B_d]
    //            [ 0    I ]
    var phi = M.times(dtSeconds).exp();

    var discA = new Matrix<States, States>(new SimpleMatrix(states, states));
    discA.extractFrom(0, 0, phi);

    var discB = new Matrix<States, Inputs>(new SimpleMatrix(states, inputs));
    discB.extractFrom(0, contB.getNumRows(), phi);

    return new Pair<>(discA, discB);
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * <p>Rather than solving a 2N x 2N matrix exponential like in DiscretizeQ() (which is expensive),
   * we take advantage of the structure of the block matrix of A and Q.
   *
   * <ul>
   *   <li>eᴬᵀ, which is only N x N, is relatively cheap.
   *   <li>The upper-right quarter of the 2N x 2N matrix, which we can approximate using a taylor
   *       series to several terms and still be substantially cheaper than taking the big
   *       exponential.
   * </ul>
   *
   * @param <States> Nat representing the number of states.
   * @param contA Continuous system matrix.
   * @param contQ Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  public static <States extends Num>
      Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQ(
          Matrix<States, States> contA, Matrix<States, States> contQ, double dtSeconds) {
    //       T
    // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
    //       0
    //
    // M = [−A  Q ]
    //     [ 0  Aᵀ]
    // ϕ = eᴹᵀ = [−A_d  A_d⁻¹Q_d]
    //           [ 0      A_dᵀ  ]
    // ϕ₁₂ = A_d⁻¹Q_d
    //
    // Taylor series of ϕ:
    //
    //   ϕ = eᴹᵀ = I + MT + 1/2 M²T² + 1/6 M³T³ + …
    //   ϕ = eᴹᵀ = I + MT + 1/2 T²M² + 1/6 T³M³ + …
    //
    // Taylor series of ϕ expanded for ϕ₁₂:
    //
    //   ϕ₁₂ = 0 + TM₁₂ + 1/2 T²M²₁₂ + 1/6 T³M³₁₂ + …
    //
    // M = [−A  Q ]
    //     [ 0  Aᵀ]
    //
    // Let B = −A, C = Q, D = Aᵀ.
    //
    //   [E  F][B  C] = [EB  EC + FD]
    //   [0  G][0  D]   [0     GD   ]
    //                = [−EA  EQ + FAᵀ]
    //                  [0       GD   ]

    // Make continuous Q symmetric if it isn't already
    Matrix<States, States> Q = contQ.plus(contQ.transpose()).div(2.0);

    Matrix<States, States> F = Q;
    Matrix<States, States> E = contA.times(-1);

    double coeff = dtSeconds;
    Matrix<States, States> phi12 = new Matrix<>(new SimpleMatrix(Q.getNumRows(), Q.getNumCols()));

    // i = 6 i.e. 5th order should be enough precision
    for (int i = 1; i < 6; ++i) {
      phi12 = phi12.plus(F.times(coeff));

      F = E.times(Q).plus(F.times(contA.transpose()));
      E = E.times(contA.times(-1));
      coeff *= dtSeconds / (double) (i + 1);
    }

    var discA = discretizeA(contA, dtSeconds);
    Q = discA.times(phi12);

    // Make Q symmetric if it isn't already
    var discQ = Q.plus(Q.transpose()).div(2.0);

    return new Pair<>(discA, discQ);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise covariance matrix.
   * Note that dt=0.0 divides R by zero.
   *
   * @param <O> Nat representing the number of outputs.
   * @param contR Continuous measurement noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return Discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeR(Matrix<O, O> contR, double dtSeconds) {
    // R_d = 1/T R
    return contR.div(dtSeconds);
  }
}
