// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.trajectory;

import org.wpilib.math.optimization.solver.ExitStatus;

/** Exception thrown to indicate optimization problem solver failure. */
public class ExitStatusException extends RuntimeException {
  /** The exit status. */
  private final ExitStatus status;

  /**
   * Constructs a new ExitStatusException with the specified exit status.
   *
   * @param status the exit status
   */
  public ExitStatusException(ExitStatus status) {
    super(status.toString());
    this.status = status;
  }

  /**
   * Gets the exit status.
   *
   * @return the exit status
   */
  public ExitStatus getExitStatus() {
    return status;
  }
}
