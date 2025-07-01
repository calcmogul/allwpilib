// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

/** A vector of inequality constraints of the form cᵢ(x) ≥ 0. */
public class InequalityConstraints implements AutoCloseable {
  private long m_handle;

  /**
   * Constructs an InequalityConstraints.
   *
   * @param handle The native handle.
   */
  public InequalityConstraints(long handle) {
    m_handle = handle;
  }

  @Override
  public void close() {
    InequalityConstraintsJNI.destroy(m_handle);
  }
}
