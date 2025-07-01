// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

/** A vector of equality constraints of the form cₑ(x) = 0. */
public class EqualityConstraints implements AutoCloseable {
  private long m_handle;

  /**
   * Constructs an EqualityConstraints.
   *
   * @param handle The native handle.
   */
  public EqualityConstraints(long handle) {
    m_handle = handle;
  }

  @Override
  public void close() {
    EqualityConstraintsJNI.destroy(m_handle);
  }
}
