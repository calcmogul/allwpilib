// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system;

import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.jni.DiscretizationJNI;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.util.Num;
import org.wpilib.util.Pair;

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
   * @param dt Discretization timestep in seconds.
   * @return the discrete matrix system.
   */
  public static <States extends Num> Matrix<States, States> discretizeA(
      Matrix<States, States> contA, double dt) {
    // A_d = eᴬᵀ
    return contA.times(dt).exp();
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param <States> Nat representing the states of the system.
   * @param <Inputs> Nat representing the inputs to the system.
   * @param contA Continuous system matrix.
   * @param contB Continuous input matrix.
   * @param dt Discretization timestep in seconds.
   * @return a Pair representing discA and diskB.
   */
  public static <States extends Num, Inputs extends Num>
      Pair<Matrix<States, States>, Matrix<States, Inputs>> discretizeAB(
          Matrix<States, States> contA, Matrix<States, Inputs> contB, double dt) {
    int states = contA.getNumRows();
    int inputs = contB.getNumCols();

    var discA = new Matrix<States, States>(new SimpleMatrix(states, states));
    var discB = new Matrix<States, Inputs>(new SimpleMatrix(states, inputs));
    DiscretizationJNI.discretizeAB(
        contA.getStorage().getDDRM().getData(),
        contB.getStorage().getDDRM().getData(),
        states,
        inputs,
        dt,
        discA.getStorage().getDDRM().getData(),
        discB.getStorage().getDDRM().getData());

    return new Pair<>(discA, discB);
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * @param <States> Nat representing the number of states.
   * @param contA Continuous system matrix.
   * @param contQ Continuous process noise covariance matrix.
   * @param dt Discretization timestep in seconds.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  public static <States extends Num>
      Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQ(
          Matrix<States, States> contA, Matrix<States, States> contQ, double dt) {
    int states = contA.getNumRows();

    var discA = new Matrix<States, States>(new SimpleMatrix(states, states));
    var discQ = new Matrix<States, States>(new SimpleMatrix(states, states));
    DiscretizationJNI.discretizeAQ(
        contA.getStorage().getDDRM().getData(),
        contQ.getStorage().getDDRM().getData(),
        states,
        dt,
        discA.getStorage().getDDRM().getData(),
        discQ.getStorage().getDDRM().getData());

    return new Pair<>(discA, discQ);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise covariance matrix.
   * Note that dt=0.0 divides R by zero.
   *
   * @param <O> Nat representing the number of outputs.
   * @param contR Continuous measurement noise covariance matrix.
   * @param dt Discretization timestep in seconds.
   * @return Discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeR(Matrix<O, O> contR, double dt) {
    // R_d = 1/T R
    return contR.div(dt);
  }
}
