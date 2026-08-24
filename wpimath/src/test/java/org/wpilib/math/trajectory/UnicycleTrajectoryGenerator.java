// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.trajectory;

import static org.wpilib.math.autodiff.Variable.cos;
import static org.wpilib.math.autodiff.Variable.sin;
import static org.wpilib.math.optimization.Constraints.bounds;
import static org.wpilib.math.optimization.Constraints.eq;
import static org.wpilib.math.optimization.Constraints.ge;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import org.wpilib.math.autodiff.Slice;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;
import org.wpilib.math.spline.QuinticHermiteSpline;
import org.wpilib.math.spline.SplineHelper;

/** Generates a trajectory through the given waypoints satisfying the given constraints. */
public final class UnicycleTrajectoryGenerator {
  private UnicycleTrajectoryGenerator() {
    // Utility class.
  }

  /**
   * Performs 4th order Runge-Kutta integration of dx/dt = f(x, u) for dt.
   *
   * @param f The function to integrate. It must take two arguments x and u.
   * @param x The initial value of x.
   * @param u The value u held constant over the integration period.
   * @param dt The time over which to integrate.
   * @return the integration of dx/dt = f(x, u) for dt.
   */
  private static VariableMatrix rk4(
      BiFunction<VariableMatrix, VariableMatrix, VariableMatrix> f,
      VariableMatrix x,
      VariableMatrix u,
      Variable dt) {
    final var h = dt;

    var k1 = f.apply(x, u);
    var k2 = f.apply(x.plus(k1.times(h.times(0.5))), u);
    var k3 = f.apply(x.plus(k2.times(h.times(0.5))), u);
    var k4 = f.apply(x.plus(k3.times(h)), u);

    return x.plus(k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4).times(h.div(6.0)));
  }

  /**
   * Unicycle drivetrain dynamics.
   *
   * @param x State vector.
   * @param u Input vector.
   * @return dx/dt.
   */
  private static VariableMatrix f(VariableMatrix x, VariableMatrix u) {
    //  x = [x, y, θ, v, ω]
    //  u = [a, α]
    //
    //  ẋ = v cosθ
    //  ẏ = v sinθ
    //  θ̇ = ω
    //  v̇ = a
    //  ω̇ = α
    var θ = x.get(2, 0);
    var v = x.get(3, 0);
    var ω = x.get(4, 0);
    var a = u.get(0, 0);
    var α = u.get(1, 0);
    return new VariableMatrix(
        new Variable[][] {{v.times(cos(θ))}, {v.times(sin(θ))}, {ω}, {a}, {α}});
  }

  /**
   * Returns the spline pose at t, or the fallback if the spline is degenerate.
   *
   * @param spline The spline.
   * @param t The spline parameter.
   * @param fallback The pose to return if the spline is degenerate at t.
   * @return The spline pose at t.
   */
  private static Pose2d getPose(QuinticHermiteSpline spline, double t, Pose2d fallback) {
    return spline.getPoint(t).map(point -> point.pose).orElse(fallback);
  }

  /**
   * Sets the state and timestep initial guess by sampling quintic Hermite splines through the
   * waypoints.
   *
   * <p>Each segment's timestep is the smallest one that keeps the finite-difference linear and
   * angular velocities within their limits. Seeding nonzero velocities keeps the solver from
   * converging to a time-reversed solution with negative timesteps.
   *
   * @param X State decision variables.
   * @param dts Timestep decision variables.
   * @param waypoints The waypoints.
   * @param N_sgmt The number of samples per segment.
   * @param maxLinearVelocity The maximum linear velocity.
   * @param maxAngularVelocity The maximum angular velocity.
   */
  private static void setInitialGuess(
      VariableMatrix X,
      VariableMatrix dts,
      List<Pose2d> waypoints,
      int N_sgmt,
      double maxLinearVelocity,
      double maxAngularVelocity) {
    var splines = SplineHelper.getQuinticSplinesFromWaypoints(waypoints);

    var poses = new ArrayList<Pose2d>();
    for (int sgmt = 0; sgmt < splines.length; ++sgmt) {
      for (int i = 0; i < N_sgmt; ++i) {
        poses.add(getPose(splines[sgmt], (double) i / N_sgmt, waypoints.get(sgmt)));
      }
    }
    poses.add(getPose(splines[splines.length - 1], 1.0, waypoints.get(waypoints.size() - 1)));

    // Unwrap heading so the guess is continuous
    double θ = waypoints.get(0).getRotation().getRadians();
    for (int k = 0; k < poses.size(); ++k) {
      final var pose = poses.get(k);
      θ += pose.getRotation().minus(new Rotation2d(θ)).getRadians();
      X.get(0, k).setValue(pose.getX());
      X.get(1, k).setValue(pose.getY());
      X.get(2, k).setValue(θ);
    }

    // Velocity and timestep initial guess
    for (int sgmt = 0; sgmt < splines.length; ++sgmt) {
      final int begin = sgmt * N_sgmt;
      final int end = begin + N_sgmt;

      double maxDistance = 0.0;
      double maxHeadingChange = 0.0;
      for (int k = begin; k < end; ++k) {
        maxDistance =
            Math.max(
                maxDistance,
                Math.hypot(
                    X.get(0, k + 1).value() - X.get(0, k).value(),
                    X.get(1, k + 1).value() - X.get(1, k).value()));
        maxHeadingChange =
            Math.max(maxHeadingChange, Math.abs(X.get(2, k + 1).value() - X.get(2, k).value()));
      }

      double dt = Math.max(maxDistance / maxLinearVelocity, maxHeadingChange / maxAngularVelocity);
      // Degenerate segments (no translation or rotation) still need a positive timestep to divide
      // by
      dt = Math.max(dt, 1e-3);

      for (int k = begin; k < end; ++k) {
        dts.get(k).setValue(dt);

        // Initial and final velocities are constrained to zero
        if (k == 0) {
          continue;
        }

        X.get(3, k)
            .setValue(
                Math.hypot(
                        X.get(0, k + 1).value() - X.get(0, k).value(),
                        X.get(1, k + 1).value() - X.get(1, k).value())
                    / dt);
        X.get(4, k).setValue((X.get(2, k + 1).value() - X.get(2, k).value()) / dt);
      }
    }
  }

  /**
   * Generates a trajectory from translation waypoints bookended by pose waypoints.
   *
   * @param initialWaypoint Initial pose waypoint.
   * @param interiorWaypoints Interior translation waypoints.
   * @param finalWaypoint Final pose waypoint.
   * @param maxLinearVelocity Linear velocity max.
   * @param maxAngularVelocity Angular velocity max.
   * @param maxLinearAcceleration Linear acceleration max.
   * @param maxAngularAcceleration Angular acceleration max.
   * @return The trajectory.
   * @throws ExitStatusException if the solver fails or the optimization problem is infeasible.
   */
  public static HolonomicTrajectory generate(
      Pose2d initialWaypoint,
      List<Translation2d> interiorWaypoints,
      Pose2d finalWaypoint,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration,
      double maxAngularAcceleration) {
    final int N_sgmt = 40;
    final int num_segments = interiorWaypoints.size() + 1;

    try (var problem = new Problem()) {
      final var X = problem.decisionVariable(5, num_segments * N_sgmt + 1);
      final var U = problem.decisionVariable(2, num_segments * N_sgmt);
      final var dts = problem.decisionVariable(num_segments * N_sgmt);

      var waypoints = new ArrayList<Translation2d>();
      waypoints.add(initialWaypoint.getTranslation());
      for (final var waypoint : interiorWaypoints) {
        waypoints.add(waypoint);
      }
      waypoints.add(finalWaypoint.getTranslation());

      // Initial guess. Interior waypoint headings point from the previous waypoint to the next
      // one.
      var poseWaypoints = new ArrayList<Pose2d>();
      poseWaypoints.add(initialWaypoint);
      for (int i = 1; i < waypoints.size() - 1; ++i) {
        poseWaypoints.add(
            new Pose2d(
                waypoints.get(i),
                waypoints
                    .get(i + 1)
                    .minus(waypoints.get(i - 1))
                    .getAngle()
                    .orElse(Rotation2d.ZERO)));
      }
      poseWaypoints.add(finalWaypoint);
      setInitialGuess(X, dts, poseWaypoints, N_sgmt, maxLinearVelocity, maxAngularVelocity);

      // dt constraints
      problem.subjectTo(ge(dts, 0));
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
          problem.subjectTo(eq(dts.get(k), dts.get(k + 1)));
        }
      }

      // Waypoint constraints
      for (int sgmt = 0; sgmt < waypoints.size(); ++sgmt) {
        final var waypoint = waypoints.get(sgmt);

        var x = X.get(0, sgmt * N_sgmt);
        var y = X.get(1, sgmt * N_sgmt);
        var θ = X.get(2, sgmt * N_sgmt);
        problem.subjectTo(eq(x, waypoint.getX()));
        problem.subjectTo(eq(y, waypoint.getY()));
        if (sgmt == 0) {
          problem.subjectTo(eq(cos(θ), initialWaypoint.getRotation().getCos()));
          problem.subjectTo(eq(sin(θ), initialWaypoint.getRotation().getSin()));
        } else if (sgmt == waypoints.size() - 1) {
          problem.subjectTo(eq(cos(θ), finalWaypoint.getRotation().getCos()));
          problem.subjectTo(eq(sin(θ), finalWaypoint.getRotation().getSin()));
        }
      }

      // Dynamics constraints
      for (int k = 0; k < X.cols() - 1; ++k) {
        var x_k = X.get(Slice.__, k);
        var x_k1 = X.get(Slice.__, k + 1);
        var u_k = U.get(Slice.__, k);
        var dt_k = dts.get(k);
        problem.subjectTo(
            eq(
                x_k1,
                rk4(
                    UnicycleTrajectoryGenerator::f,
                    new VariableMatrix(x_k),
                    new VariableMatrix(u_k),
                    dt_k)));
      }

      // Initial and final velocities
      problem.subjectTo(eq(X.get(new Slice(3, 5), 0), 0));
      problem.subjectTo(eq(X.get(new Slice(3, 5), -1), 0));

      // Velocity limits
      problem.subjectTo(bounds(-maxLinearVelocity, X.get(3, Slice.__), maxLinearVelocity));
      problem.subjectTo(bounds(-maxAngularVelocity, X.get(4, Slice.__), maxAngularVelocity));

      // Acceleration limits
      problem.subjectTo(bounds(-maxLinearAcceleration, U.get(0, Slice.__), maxLinearAcceleration));
      problem.subjectTo(
          bounds(-maxAngularAcceleration, U.get(1, Slice.__), maxAngularAcceleration));

      problem.minimize(dts.stream().reduce(new Variable(0.0), (a, b) -> a.plus(b)));

      var status = problem.solve();
      if (status != ExitStatus.SUCCESS) {
        throw new ExitStatusException(status);
      }

      var samples = new ArrayList<HolonomicSample>();
      double t = 0.0;
      for (int k = 0; k < X.cols(); ++k) {
        double x = X.get(0, k).value();
        double y = X.get(1, k).value();
        var θ = new Rotation2d(X.get(2, k).value());
        double v = X.get(3, k).value();
        double ω = X.get(4, k).value();

        if (k < X.cols() - 1) {
          double a = U.get(0, k).value();
          double α = U.get(1, k).value();
          double dt = dts.get(k).value();

          // The robot-relative acceleration includes the centripetal term vω
          double a_c = v * ω;

          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, θ),
                  new ChassisVelocities(v, 0.0, ω).toFieldRelative(θ),
                  new ChassisAccelerations(a, a_c, α).toFieldRelative(θ)));
          t += dt;
        } else {
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, θ),
                  new ChassisVelocities(v, 0.0, ω).toFieldRelative(θ),
                  new ChassisAccelerations(0.0, 0.0, 0.0).toFieldRelative(θ)));
        }
      }

      return new HolonomicTrajectory(samples);
    }
  }

  /**
   * Generates a trajectory from pose waypoints.
   *
   * @param waypoints Pose waypoints.
   * @param maxLinearVelocity Linear velocity max.
   * @param maxAngularVelocity Angular velocity max.
   * @param maxLinearAcceleration Linear acceleration max.
   * @param maxAngularAcceleration Angular acceleration max.
   * @return The trajectory.
   * @throws ExitStatusException if the solver fails or the optimization problem is infeasible.
   */
  public static HolonomicTrajectory generate(
      List<Pose2d> waypoints,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration,
      double maxAngularAcceleration) {
    final int N_sgmt = 40;
    final int num_segments = waypoints.size() - 1;

    try (var problem = new Problem()) {
      final var X = problem.decisionVariable(5, num_segments * N_sgmt + 1);
      final var U = problem.decisionVariable(2, num_segments * N_sgmt);
      final var dts = problem.decisionVariable(num_segments * N_sgmt);

      // Initial guess
      setInitialGuess(X, dts, waypoints, N_sgmt, maxLinearVelocity, maxAngularVelocity);

      // dt constraints
      problem.subjectTo(ge(dts, 0));
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
          problem.subjectTo(eq(dts.get(k), dts.get(k + 1)));
        }
      }

      // Waypoint constraints
      for (int sgmt = 0; sgmt < waypoints.size(); ++sgmt) {
        final var waypoint = waypoints.get(sgmt);

        var x = X.get(0, sgmt * N_sgmt);
        var y = X.get(1, sgmt * N_sgmt);
        var θ = X.get(2, sgmt * N_sgmt);
        problem.subjectTo(eq(x, waypoint.getX()));
        problem.subjectTo(eq(y, waypoint.getY()));
        problem.subjectTo(eq(cos(θ), waypoint.getRotation().getCos()));
        problem.subjectTo(eq(sin(θ), waypoint.getRotation().getSin()));
      }

      // Dynamics constraints
      for (int k = 0; k < X.cols() - 1; ++k) {
        var x_k = X.get(Slice.__, k);
        var x_k1 = X.get(Slice.__, k + 1);
        var u_k = U.get(Slice.__, k);
        var dt_k = dts.get(k);
        problem.subjectTo(
            eq(
                x_k1,
                rk4(
                    UnicycleTrajectoryGenerator::f,
                    new VariableMatrix(x_k),
                    new VariableMatrix(u_k),
                    dt_k)));
      }

      // Initial and final velocities
      problem.subjectTo(eq(X.get(new Slice(3, 5), 0), 0));
      problem.subjectTo(eq(X.get(new Slice(3, 5), -1), 0));

      // Velocity limits
      problem.subjectTo(bounds(-maxLinearVelocity, X.get(3, Slice.__), maxLinearVelocity));
      problem.subjectTo(bounds(-maxAngularVelocity, X.get(4, Slice.__), maxAngularVelocity));

      // Acceleration limits
      problem.subjectTo(bounds(-maxLinearAcceleration, U.get(0, Slice.__), maxLinearAcceleration));
      problem.subjectTo(
          bounds(-maxAngularAcceleration, U.get(1, Slice.__), maxAngularAcceleration));

      problem.minimize(dts.stream().reduce(new Variable(0.0), (a, b) -> a.plus(b)));

      var status = problem.solve();
      if (status != ExitStatus.SUCCESS) {
        throw new ExitStatusException(status);
      }

      var samples = new ArrayList<HolonomicSample>();
      double t = 0.0;
      for (int k = 0; k < X.cols(); ++k) {
        double x = X.get(0, k).value();
        double y = X.get(1, k).value();
        var θ = new Rotation2d(X.get(2, k).value());
        double v = X.get(3, k).value();
        double ω = X.get(4, k).value();

        if (k < X.cols() - 1) {
          double a = U.get(0, k).value();
          double α = U.get(1, k).value();
          double dt = dts.get(k).value();

          // The robot-relative acceleration includes the centripetal term vω
          double a_c = v * ω;

          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, θ),
                  new ChassisVelocities(v, 0.0, ω).toFieldRelative(θ),
                  new ChassisAccelerations(a, a_c, α).toFieldRelative(θ)));
          t += dt;
        } else {
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, θ),
                  new ChassisVelocities(v, 0.0, ω).toFieldRelative(θ),
                  new ChassisAccelerations(0.0, 0.0, 0.0).toFieldRelative(θ)));
        }
      }

      return new HolonomicTrajectory(samples);
    }
  }
}
