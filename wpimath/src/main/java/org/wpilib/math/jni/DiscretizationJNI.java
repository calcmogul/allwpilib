// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.jni;

/** Discretization JNI. */
public final class DiscretizationJNI extends WPIMathJNI {
  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param contA Array containing elements of continuous A in row-major order.
   * @param contB Array containing elements of continuous B in row-major order.
   * @param states Number of states in A matrix.
   * @param inputs Number of inputs in B matrix.
   * @param dt Discretization timestep in seconds.
   * @param discA Array storage for discrete A in row-major order.
   * @param discB Array storage for discrete B in row-major order.
   */
  public static native void discretizeAB(
      double[] contA,
      double[] contB,
      int states,
      int inputs,
      double dt,
      double[] discA,
      double[] discB);

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * @param contA Array containing elements of continuous A in row-major order.
   * @param contQ Array containing elements of continuous Q in row-major order.
   * @param states Number of states in A matrix.
   * @param dt Discretization timestep in seconds.
   * @param discA Array storage for discrete A in row-major order.
   * @param discQ Array storage for discrete Q in row-major order.
   */
  public static native void discretizeAQ(
      double[] contA, double[] contQ, int states, double dt, double[] discA, double[] discQ);

  /** Utility class. */
  private DiscretizationJNI() {}
}
