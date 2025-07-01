// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.optimization;

import edu.wpi.first.math.autodiff.ExpressionType;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.math.optimization.solver.ExitStatus;
import edu.wpi.first.math.optimization.solver.Options;

/** Problem JNI functions. */
public class ProblemJNI extends WPIMathJNI {
  /** Construct the optimization problem. */
  static native long create();

  /**
   * Destruct the optimization problem.
   *
   * @param handle Problem handle.
   */
  static native void destroy(long handle);

  /**
   * Create a decision variable in the optimization problem.
   *
   * @param handle Problem handle.
   * @return A decision variable in the optimization problem.
   */
  static native long decisionVariableScalar(long handle);

  /**
   * Create a column vector of decision variables in the optimization problem.
   *
   * @param handle Problem handle.
   * @param rows Number of matrix rows.
   * @return A matrix of decision variables in the optimization problem.
   */
  static native long decisionVariableColumnVector(long handle, int rows);

  /**
   * Create a matrix of decision variables in the optimization problem.
   *
   * @param handle Problem handle.
   * @param rows Number of matrix rows.
   * @param cols Number of matrix columns.
   * @return A matrix of decision variables in the optimization problem.
   */
  static native long decisionVariableMatrix(long handle, int rows, int cols);

  /**
   * Create a symmetric matrix of decision variables in the optimization problem.
   *
   * <p>Variable instances are reused across the diagonal, which helps reduce problem
   * dimensionality.
   *
   * @param handle Problem handle.
   * @param rows Number of matrix rows.
   * @return A symmetric matrix of decision varaibles in the optimization problem.
   */
  static native long symmetricDecisionVariable(long handle, int rows);

  /**
   * Tells the solver to minimize the output of the given cost function.
   *
   * <p>Note that this is optional. If only constraints are specified, the solver will find the
   * closest solution to the initial conditions that's in the feasible set.
   *
   * @param handle Problem handle.
   * @param costHandle Variable handle of the cost function to minimize.
   */
  static native void minimize(long handle, long costHandle);

  /**
   * Tells the solver to maximize the output of the given objective function.
   *
   * <p>Note that this is optional. If only constraints are specified, the solver will find the
   * closest solution to the initial conditions that's in the feasible set.
   *
   * @param handle Problem handle.
   * @param objectiveHandle Variable handle of the objective function to maximize.
   */
  static native void maximize(long handle, long objectiveHandle);

  /**
   * Tells the solver to solve the problem while satisfying the given equality constraint.
   *
   * @param handle Problem handle.
   * @param constraint The constraint to satisfy.
   */
  static native void subjectToEq(long handle, EqualityConstraints constraint);

  /**
   * Tells the solver to solve the problem while satisfying the given inequality constraint.
   *
   * @param handle Problem handle.
   * @param constraint The constraint to satisfy.
   */
  static native void subjectToIneq(long handle, InequalityConstraints constraint);

  /**
   * Returns the cost function's type.
   *
   * @param handle Problem handle.
   * @return The cost function's type.
   */
  static native ExpressionType costFunctionType(long handle);

  /**
   * Returns the type of the highest order equality constraint.
   *
   * @param handle Problem handle.
   * @return The type of the highest order equality constraint.
   */
  static native ExpressionType equalityConstraintType(long handle);

  /**
   * Returns the type of the highest order inequality constraint.
   *
   * @param handle Problem handle.
   * @return The type of the highest order inequality constraint.
   */
  static native ExpressionType inequalityConstraintType(long handle);

  /**
   * Solve the optimization problem. The solution will be stored in the original variables used to
   * construct the problem.
   *
   * @param handle Problme handle.
   * @return The solver status.
   */
  static native ExitStatus solve(long handle);

  /**
   * Solve the optimization problem. The solution will be stored in the original variables used to
   * construct the problem.
   *
   * @param handle Problem handle.
   * @param options Solver options.
   * @return The solver status.
   */
  static native ExitStatus solveWithOptions(long handle, Options options);

  /**
   * Solve the optimization problem. The solution will be stored in the original variables used to
   * construct the problem.
   *
   * @param handle Problem handle.
   * @param options Solver options.
   * @param spy Enables writing sparsity patterns of H, Aₑ, and Aᵢ to files named H.spy, A_e.spy,
   *     and A_i.spy respectively during solve. Use tools/spy.py to plot them.
   * @return The solver status.
   */
  static native ExitStatus solveWithOptionsSpy(long handle, Options options, boolean spy);
}
