// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

/** Constraint creation helper functions. */
public class Constraints {
  /** Utility class. */
  private Constraints() {}

  /**
   * Equality operator that returns an equality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static EqualityConstraints eq(double lhs, Variable rhs) {
    return eq(new Variable(lhs), rhs);
  }

  /**
   * Equality operator that returns an equality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static EqualityConstraints eq(Variable lhs, double rhs) {
    return eq(lhs, new Variable(rhs));
  }

  /**
   * Equality operator that returns an equality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static EqualityConstraints eq(Variable lhs, Variable rhs) {
    return new EqualityConstraints(ConstraintsJNI.eq(lhs.getHandle(), rhs.getHandle()));
  }

  /**
   * Less-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints lt(double lhs, Variable rhs) {
    return lt(new Variable(lhs), rhs);
  }

  /**
   * Less-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints lt(Variable lhs, double rhs) {
    return lt(lhs, new Variable(rhs));
  }

  /**
   * Less-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints lt(Variable lhs, Variable rhs) {
    return new InequalityConstraints(ConstraintsJNI.lt(lhs.getHandle(), rhs.getHandle()));
  }

  /**
   * Less-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints le(double lhs, Variable rhs) {
    return le(new Variable(lhs), rhs);
  }

  /**
   * Less-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints le(Variable lhs, double rhs) {
    return le(lhs, new Variable(rhs));
  }

  /**
   * Less-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints le(Variable lhs, Variable rhs) {
    return new InequalityConstraints(ConstraintsJNI.le(lhs.getHandle(), rhs.getHandle()));
  }

  /**
   * Greater-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints gt(double lhs, Variable rhs) {
    return gt(new Variable(lhs), rhs);
  }

  /**
   * Greater-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints gt(Variable lhs, double rhs) {
    return gt(lhs, new Variable(rhs));
  }

  /**
   * Greater-than comparison operator that returns an inequality constraint for two Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints gt(Variable lhs, Variable rhs) {
    return new InequalityConstraints(ConstraintsJNI.gt(lhs.getHandle(), rhs.getHandle()));
  }

  /**
   * Greater-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints ge(double lhs, Variable rhs) {
    return ge(new Variable(lhs), rhs);
  }

  /**
   * Greater-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints ge(Variable lhs, double rhs) {
    return ge(lhs, new Variable(rhs));
  }

  /**
   * Greater-than-or-equal-to comparison operator that returns an inequality constraint for two
   * Variables.
   *
   * @param lhs Left-hand side.
   * @param rhs Left-hand side.
   */
  public static InequalityConstraints ge(Variable lhs, Variable rhs) {
    return new InequalityConstraints(ConstraintsJNI.ge(lhs.getHandle(), rhs.getHandle()));
  }
}
