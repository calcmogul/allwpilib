// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.autodiff.Jacobian;
import edu.wpi.first.math.autodiff.Variable;
import edu.wpi.first.math.autodiff.VariableBlock;
import edu.wpi.first.math.autodiff.VariableMatrix;
import org.ejml.data.DMatrixSparseCSC;

/**
 * This class calculates the gradient of a variable with respect to a vector of
 * variables.
 *
 * The gradient is only recomputed if the variable expression is quadratic or
 * higher order.
 */
public class Gradient implements AutoCloseable {
  private int m_handle;

  /**
   * Constructs a Gradient object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Variable with respect to which to compute the gradient.
   */
  public Gradient(Variable variable, Variable wrt) {
    m_handle = GradientJNI.create(variable, wrt);
  }

  /**
   * Constructs a Gradient object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Vector of variables with respect to which to compute the
   *   gradient.
   */
  public Gradient(Variable variable, VariableMatrix wrt) {
    m_handle = GradientJNI.create(new VariableMatrix(variable), wrt);
  }

  /**
   * Constructs a Gradient object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Vector of variables with respect to which to compute the
   *   gradient.
   */
  public Gradient(Variable variable, VariableBlock wrt) {
    m_handle = GradientJNI.create(new VariableMatrix(variable), wrt);
  }

  @Override
  public void close() {
    GradientJNI.destroy(m_handle);
  }

  /**
   * Returns the gradient as a VariableMatrix.
   *
   * This is useful when constructing optimization problems with derivatives in
   * them.
   *
   * @return The gradient as a VariableMatrix.
   */
  public VariableMatrix get() { return GradientJNI.get(m_handle); }

  /**
   * Evaluates the gradient at wrt's value.
   *
   * @return The gradient at wrt's value.
   */
  public DMatrixSparseCSC value() {
    return GradientJNI.value(m_handle);
  }
}
