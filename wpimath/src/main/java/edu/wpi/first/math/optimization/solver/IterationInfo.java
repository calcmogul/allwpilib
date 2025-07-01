// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.ejml.data.DMatrixSparseCSC;
import org.ejml.simple.SimpleMatrix;

/**
 * Solver iteration information exposed to an iteration callback.
 */
public class IterationInfo {
  /** The solver iteration. */
  int iteration;

  /** The decision variables. */
  SimpleMatrix x;

  /** The gradient of the cost function. */
  DMatrixSparseCSC g;

  /** The Hessian of the Lagrangian. */
  DMatrixSparseCSC H;

  /** The equality constraint Jacobian. */
  DMatrixSparseCSC A_e;

  /** The inequality constraint Jacobian. */
  DMatrixSparseCSC A_i;

  /**
   * Constructs iteration info.
   *
   * @param iterations The solver iteration.
   * @param x The decision variables.
   * @param g The gradient of the cost function.
   * @param H The Hessian of the Lagrangian.
   * @param A_e The equality constraint Jacobian.
   * @param A_i The inequality constraint Jacobian.
   */
  public IterationInfo(int iteration, SimpleMatrix x, DMatrixSparseCSC g, DMatrixSparseCSC H, DMatrixSparseCSC A_e, DMatrixSparseCSC A_i) {
    this.iteration = iteration;
    this.x = x;
    this.g = g;
    this.H = H;
    this.A_e = A_e;
    this.A_i = A_i;
  }
}
