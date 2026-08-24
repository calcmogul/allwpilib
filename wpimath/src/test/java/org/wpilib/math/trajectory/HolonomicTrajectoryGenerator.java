// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.trajectory;

import static org.wpilib.math.autodiff.Variable.cos;
import static org.wpilib.math.autodiff.Variable.sin;
import static org.wpilib.math.optimization.Constraints.bounds;
import static org.wpilib.math.optimization.Constraints.eq;

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
import org.wpilib.math.util.MathUtil;

/** Generates a holonomic drivetrain trajectory using trajectory optimization. */
public final class HolonomicTrajectoryGenerator {
  private HolonomicTrajectoryGenerator() {
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
  static VariableMatrix rk4(
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
   * Generates a trajectory from translation waypoints bookended by pose waypoints.
   *
   * @param initialWaypoint Initial pose waypoint.
   * @param interiorWaypoints Interior translation waypoints.
   * @param finalWaypoint Final pose waypoint.
   * @param maxLinearVelocity Linear velocity max.
   * @param maxAngularVelocity Angular velocity max.
   * @param maxLinearAcceleration Linear acceleration max.
   * @param maxAngularAcceleration Angular acceleration max.
   * @return Trajectory.
   */
  public static HolonomicTrajectory generate(
      Pose2d initialWaypoint,
      List<Translation2d> interiorWaypoints,
      Pose2d finalWaypoint,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration,
      double maxAngularAcceleration) {
    BiFunction<VariableMatrix, VariableMatrix, VariableMatrix> f =
        (VariableMatrix x, VariableMatrix u) -> {
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
        };

    final int N_sgmt = 40;
    final int num_segments = interiorWaypoints.size() + 1;
    final double T_max = 5.0;

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

      // Pose initial guess
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1); ++k) {
          double scale = (double) (k - N_sgmt * sgmt) / N_sgmt;
          X.get(0, k)
              .setValue(
                  MathUtil.lerp(waypoints.get(sgmt).getX(), waypoints.get(sgmt + 1).getX(), scale));
          X.get(1, k)
              .setValue(
                  MathUtil.lerp(waypoints.get(sgmt).getY(), waypoints.get(sgmt + 1).getY(), scale));
          X.get(2, k)
              .setValue(
                  waypoints
                      .get(sgmt + 1)
                      .minus(waypoints.get(sgmt))
                      .getAngle()
                      .orElse(Rotation2d.ZERO)
                      .getRadians());
        }
      }
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        X.get(0, N_sgmt * sgmt).setValue(waypoints.get(sgmt).getX());
        X.get(1, N_sgmt * sgmt).setValue(waypoints.get(sgmt).getY());
        if (sgmt == 0) {
          X.get(2, N_sgmt * sgmt).setValue(initialWaypoint.getRotation().getRadians());
        } else if (sgmt == num_segments - 1) {
          X.get(2, N_sgmt * sgmt).setValue(finalWaypoint.getRotation().getRadians());
        }
      }
      X.get(0, X.cols() - 1).setValue(finalWaypoint.getX());
      X.get(1, X.cols() - 1).setValue(finalWaypoint.getY());
      X.get(2, X.cols() - 1).setValue(finalWaypoint.getRotation().getRadians());

      // dt constraints
      problem.subjectTo(bounds(0, dts, T_max / (num_segments * N_sgmt)));
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
          problem.subjectTo(eq(dts.get(k), dts.get(k + 1)));
        }
      }
      for (int k = 0; k < X.cols() - 1; ++k) {
        dts.get(k).setValue(T_max / N_sgmt);
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
        problem.subjectTo(eq(x_k1, rk4(f, new VariableMatrix(x_k), new VariableMatrix(u_k), dt_k)));
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

      problem.solve();

      var samples = new ArrayList<HolonomicSample>();
      double t = 0.0;
      for (int k = 0; k < X.cols(); ++k) {
        double x = X.get(0, k).value();
        double y = X.get(1, k).value();
        double θ = X.get(2, k).value();
        double v_x = X.get(3, k).value();
        double ω = X.get(4, k).value();

        if (k < X.cols() - 1) {
          double a_x = U.get(0, k).value();
          double α = U.get(1, k).value();
          double dt = dts.get(k).value();
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, new Rotation2d(θ)),
                  new ChassisVelocities(v_x, 0.0, ω),
                  new ChassisAccelerations(a_x, 0.0, α)));
          t += dt;
        } else {
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, new Rotation2d(θ)),
                  new ChassisVelocities(v_x, 0.0, ω),
                  new ChassisAccelerations(0.0, 0.0, 0.0)));
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
   * @return Trajectory.
   */
  public static HolonomicTrajectory generate(
      List<Pose2d> waypoints,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration,
      double maxAngularAcceleration) {
    BiFunction<VariableMatrix, VariableMatrix, VariableMatrix> f =
        (VariableMatrix x, VariableMatrix u) -> {
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
        };

    final int N_sgmt = 40;
    final int num_segments = waypoints.size() - 1;
    final double T_max = 5.0;

    try (var problem = new Problem()) {
      final var X = problem.decisionVariable(5, num_segments * N_sgmt + 1);
      final var U = problem.decisionVariable(2, num_segments * N_sgmt);
      final var dts = problem.decisionVariable(num_segments * N_sgmt);

      // Pose initial guess
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1); ++k) {
          double scale = (double) (k - N_sgmt * sgmt) / N_sgmt;
          X.get(0, k)
              .setValue(
                  MathUtil.lerp(waypoints.get(sgmt).getX(), waypoints.get(sgmt + 1).getX(), scale));
          X.get(1, k)
              .setValue(
                  MathUtil.lerp(waypoints.get(sgmt).getY(), waypoints.get(sgmt + 1).getY(), scale));
          X.get(2, k)
              .setValue(
                  waypoints
                      .get(sgmt + 1)
                      .getTranslation()
                      .minus(waypoints.get(sgmt).getTranslation())
                      .getAngle()
                      .orElse(Rotation2d.ZERO)
                      .getRadians());
        }
      }
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        X.get(0, N_sgmt * sgmt).setValue(waypoints.get(sgmt).getX());
        X.get(1, N_sgmt * sgmt).setValue(waypoints.get(sgmt).getY());
        X.get(2, N_sgmt * sgmt).setValue(waypoints.get(sgmt).getRotation().getRadians());
      }
      X.get(0, X.cols() - 1).setValue(waypoints.get(waypoints.size() - 1).getX());
      X.get(1, X.cols() - 1).setValue(waypoints.get(waypoints.size() - 1).getY());
      X.get(2, X.cols() - 1)
          .setValue(waypoints.get(waypoints.size() - 1).getRotation().getRadians());

      // dt constraints
      problem.subjectTo(bounds(0, dts, T_max / (num_segments * N_sgmt)));
      for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
        for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
          problem.subjectTo(eq(dts.get(k), dts.get(k + 1)));
        }
      }
      for (int k = 0; k < X.cols() - 1; ++k) {
        dts.get(k).setValue(T_max / N_sgmt);
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
        problem.subjectTo(eq(x_k1, rk4(f, new VariableMatrix(x_k), new VariableMatrix(u_k), dt_k)));
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

      problem.solve();

      var samples = new ArrayList<HolonomicSample>();
      double t = 0.0;
      for (int k = 0; k < X.cols(); ++k) {
        double x = X.get(0, k).value();
        double y = X.get(1, k).value();
        double θ = X.get(2, k).value();
        double v_x = X.get(3, k).value();
        double ω = X.get(4, k).value();
        double a_x = U.get(0, k).value();
        double α = U.get(1, k).value();
        double dt = dts.get(k).value();

        if (k < X.cols() - 1) {
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, new Rotation2d(θ)),
                  new ChassisVelocities(v_x, 0.0, ω),
                  new ChassisAccelerations(a_x, 0.0, α)));
          t += dt;
        } else {
          samples.add(
              new HolonomicSample(
                  t,
                  new Pose2d(x, y, new Rotation2d(θ)),
                  new ChassisVelocities(v_x, 0.0, ω),
                  new ChassisAccelerations(0.0, 0.0, 0.0)));
        }
      }

      return new HolonomicTrajectory(samples);
    }
  }
}
