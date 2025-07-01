// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import edu.wpi.first.math.jni.WPIMathJNI;
import org.ejml.data.DMatrixSparseCSC;

/** Jacobian JNI functions. */
public class JacobianJNI extends WPIMathJNI {
  /**
   * Constructs a Jacobian object.
   *
   * @param variables Vector of variables of which to compute the Jacobian.
   * @param wrt Vector of variables with respect to which to compute the Jacobian.
   */
  static native long createSM(long variable, long[] wrt);

  /**
   * Constructs a Jacobian object.
   *
   * @param variables Vector of variables of which to compute the Jacobian.
   * @param wrt Vector of variables with respect to which to compute the Jacobian.
   */
  static native long createMM(long[] variables, long[] wrt);

  /**
   * Destructs a Jacobian.
   *
   * @param handle Jacobian handle.
   */
  static native void destroy(long handle);

  /**
   * Returns the Jacobian as a VariableMatrix.
   *
   * <p>This is useful when constructing optimization problems with derivatives in them.
   *
   * @param handle Jacobian handle.
   * @return The Jacobian as a VariableMatrix.
   */
  static native VariableMatrix get(long handle);

  /**
   * Evaluates the Jacobian at wrt's value.
   *
   * @param handle Jacobian handle.
   * @return The Jacobian at wrt's value.
   */
  static native DMatrixSparseCSC value(long handle);
}
