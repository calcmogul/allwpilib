// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory.constraint;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * An interface for defining user-defined velocity and acceleration constraints while generating
 * trajectories.
 */
public interface TrajectoryConstraint {
  /**
   * Returns the max velocity given the current pose and curvature.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param velocity The velocity at the current point in the trajectory before constraints are
   *     applied.
   * @return The absolute maximum velocity.
   */
  double getMaxVelocity(Pose2d pose, double curvature, double velocity);

  /**
   * Returns the minimum and maximum allowable acceleration for the trajectory given pose,
   * curvature, and speed.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param velocity The speed at the current point in the trajectory.
   * @return The min and max acceleration bounds.
   */
  MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity);

  /** Represents a minimum and maximum acceleration. */
  class MinMax {
    /** The minimum acceleration. */
    public double minAcceleration = -Double.MAX_VALUE;

    /** The maximum acceleration. */
    public double maxAcceleration = Double.MAX_VALUE;

    /**
     * Constructs a MinMax.
     *
     * @param minAcceleration The minimum acceleration.
     * @param maxAcceleration The maximum acceleration.
     */
    public MinMax(double minAcceleration, double maxAcceleration) {
      this.minAcceleration = minAcceleration;
      this.maxAcceleration = maxAcceleration;
    }

    /** Constructs a MinMax with default values. */
    public MinMax() {}
  }
}
