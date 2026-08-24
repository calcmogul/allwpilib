// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/trajectory/UnicycleTrajectoryGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <expected>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include <sleipnir/autodiff/slice.hpp>
#include <sleipnir/autodiff/variable.hpp>
#include <sleipnir/autodiff/variable_matrix.hpp>
#include <sleipnir/optimization/problem.hpp>
#include <sleipnir/optimization/solver/exit_status.hpp>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/kinematics/ChassisAccelerations.hpp"
#include "wpi/math/kinematics/ChassisVelocities.hpp"
#include "wpi/math/spline/SplineHelper.hpp"
#include "wpi/math/trajectory/HolonomicSample.hpp"
#include "wpi/math/trajectory/HolonomicTrajectory.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/curvature.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/units/velocity.hpp"

namespace {

/**
 * Performs 4th order Runge-Kutta integration of dx/dt = f(x, u) for dt.
 *
 * @param f  The function to integrate. It must take two arguments x and u.
 * @param x  The initial value of x.
 * @param u  The value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F>
slp::VariableMatrix<double> RK4(F&& f, const slp::VariableMatrix<double>& x,
                                const slp::VariableMatrix<double>& u,
                                const slp::Variable<double>& dt) {
  const auto h = dt;

  auto k1 = f(x, u);
  auto k2 = f(x + h * 0.5 * k1, u);
  auto k3 = f(x + h * 0.5 * k2, u);
  auto k4 = f(x + h * k3, u);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Unicycle drivetrain dynamics.
 *
 * @param x State vector.
 * @param u Input vector.
 * @return dx/dt.
 */
slp::VariableMatrix<double> f(const slp::VariableMatrix<double>& x,
                              const slp::VariableMatrix<double>& u) {
  //  x = [x, y, θ, v, ω]
  //  u = [a, α]
  //
  //  ẋ = v cosθ
  //  ẏ = v sinθ
  //  θ̇ = ω
  //  v̇ = a
  //  ω̇ = α
  auto θ = x[2, 0];
  auto v = x[3, 0];
  auto ω = x[4, 0];
  auto a = u[0, 0];
  auto α = u[1, 0];
  return slp::VariableMatrix({{v * cos(θ)}, {v * sin(θ)}, {ω}, {a}, {α}});
}

/**
 * Sets the state and timestep initial guess by sampling quintic Hermite splines
 * through the waypoints.
 *
 * Each segment's timestep is the smallest one that keeps the finite-difference
 * linear and angular velocities within their limits. Seeding nonzero
 * velocities keeps the solver from converging to a time-reversed solution with
 * negative timesteps.
 *
 * @param X State decision variables.
 * @param dts Timestep decision variables.
 * @param waypoints The waypoints.
 * @param N_sgmt The number of samples per segment.
 * @param maxLinearVelocity The maximum linear velocity.
 * @param maxAngularVelocity The maximum angular velocity.
 */
void SetInitialGuess(slp::VariableMatrix<double>& X,
                     slp::VariableMatrix<double>& dts,
                     const std::vector<wpi::math::Pose2d>& waypoints,
                     int N_sgmt,
                     wpi::units::meters_per_second_t maxLinearVelocity,
                     wpi::units::radians_per_second_t maxAngularVelocity) {
  auto splines =
      wpi::math::SplineHelper::QuinticSplinesFromWaypoints(waypoints);

  // Unwrap heading so the guess is continuous
  double θ = waypoints.front().Rotation().Radians().value();

  auto setPose = [&](int k, const wpi::math::Pose2d& pose) {
    θ += (pose.Rotation() - wpi::math::Rotation2d{wpi::units::radian_t{θ}})
             .Radians()
             .value();
    X[0, k].set_value(pose.X().value());
    X[1, k].set_value(pose.Y().value());
    X[2, k].set_value(θ);
  };

  // Returns the spline pose at t, or the fallback if the spline is degenerate
  auto getPose = [](const wpi::math::QuinticHermiteSpline& spline, double t,
                    const wpi::math::Pose2d& fallback) {
    return spline.GetPoint(t)
        .value_or(std::pair{fallback, wpi::units::curvature_t{0}})
        .first;
  };

  for (size_t sgmt = 0; sgmt < splines.size(); ++sgmt) {
    for (int i = 0; i < N_sgmt; ++i) {
      setPose(sgmt * N_sgmt + i,
              getPose(splines[sgmt], static_cast<double>(i) / N_sgmt,
                      waypoints[sgmt]));
    }
  }
  setPose(X.cols() - 1, getPose(splines.back(), 1.0, waypoints.back()));

  // Velocity and timestep initial guess
  for (size_t sgmt = 0; sgmt < splines.size(); ++sgmt) {
    const int begin = sgmt * N_sgmt;
    const int end = begin + N_sgmt;

    double maxDistance = 0.0;
    double maxHeadingChange = 0.0;
    for (int k = begin; k < end; ++k) {
      maxDistance = std::max(maxDistance,
                             std::hypot(X[0, k + 1].value() - X[0, k].value(),
                                        X[1, k + 1].value() - X[1, k].value()));
      maxHeadingChange = std::max(
          maxHeadingChange, std::abs(X[2, k + 1].value() - X[2, k].value()));
    }

    double dt = std::max(maxDistance / maxLinearVelocity.value(),
                         maxHeadingChange / maxAngularVelocity.value());
    // Degenerate segments (no translation or rotation) still need a positive
    // timestep to divide by
    dt = std::max(dt, 1e-3);

    for (int k = begin; k < end; ++k) {
      dts[k].set_value(dt);

      // Initial and final velocities are constrained to zero
      if (k == 0) {
        continue;
      }

      X[3, k].set_value(std::hypot(X[0, k + 1].value() - X[0, k].value(),
                                   X[1, k + 1].value() - X[1, k].value()) /
                        dt);
      X[4, k].set_value((X[2, k + 1].value() - X[2, k].value()) / dt);
    }
  }
}

}  // namespace

namespace wpi::math {

std::expected<HolonomicTrajectory, slp::ExitStatus>
UnicycleTrajectoryGenerator::Generate(
    const Pose2d& initialWaypoint,
    std::span<const Translation2d> interiorWaypoints,
    const Pose2d& finalWaypoint,
    wpi::units::meters_per_second_t maxLinearVelocity,
    wpi::units::radians_per_second_t maxAngularVelocity,
    wpi::units::meters_per_second_squared_t maxLinearAcceleration,
    wpi::units::radians_per_second_squared_t maxAngularAcceleration) {
  using namespace slp::slicing;

  constexpr int N_sgmt = 40;
  const int num_segments = interiorWaypoints.size() + 1;

  auto problem = slp::Problem<double>();

  auto X = problem.decision_variable(5, num_segments * N_sgmt + 1);
  auto U = problem.decision_variable(2, num_segments * N_sgmt);
  auto dts = problem.decision_variable(num_segments * N_sgmt);

  std::vector<Translation2d> waypoints;
  waypoints.emplace_back(initialWaypoint.Translation());
  for (const auto& waypoint : interiorWaypoints) {
    waypoints.emplace_back(waypoint);
  }
  waypoints.emplace_back(finalWaypoint.Translation());

  // Initial guess. Interior waypoint headings point from the previous
  // waypoint to the next one.
  std::vector<Pose2d> poseWaypoints;
  poseWaypoints.emplace_back(initialWaypoint);
  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    poseWaypoints.emplace_back(
        waypoints[i],
        (waypoints[i + 1] - waypoints[i - 1]).Angle().value_or(0_rad));
  }
  poseWaypoints.emplace_back(finalWaypoint);
  SetInitialGuess(X, dts, poseWaypoints, N_sgmt, maxLinearVelocity,
                  maxAngularVelocity);

  // dt constraints
  problem.subject_to(dts >= 0);
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
      problem.subject_to(dts[k] == dts[k + 1]);
    }
  }

  // Waypoint constraints
  for (size_t sgmt = 0; sgmt < waypoints.size(); ++sgmt) {
    const auto& waypoint = waypoints[sgmt];

    auto x = X[0, sgmt * N_sgmt];
    auto y = X[1, sgmt * N_sgmt];
    auto θ = X[2, sgmt * N_sgmt];
    problem.subject_to(x == waypoint.X().value());
    problem.subject_to(y == waypoint.Y().value());
    if (sgmt == 0) {
      problem.subject_to(cos(θ) == initialWaypoint.Rotation().Cos());
      problem.subject_to(sin(θ) == initialWaypoint.Rotation().Sin());
    } else if (sgmt == waypoints.size() - 1) {
      problem.subject_to(cos(θ) == finalWaypoint.Rotation().Cos());
      problem.subject_to(sin(θ) == finalWaypoint.Rotation().Sin());
    }
  }

  // Dynamics constraints
  for (int k = 0; k < X.cols() - 1; ++k) {
    auto x_k = X[_, k];
    auto x_k1 = X[_, k + 1];
    auto u_k = U[_, k];
    auto dt_k = dts[k];
    problem.subject_to(x_k1 == RK4(f, x_k, u_k, dt_k));
  }

  // Initial and final velocities
  problem.subject_to(X[slp::Slice{3, 5}, 0] == 0);
  problem.subject_to(X[slp::Slice{3, 5}, -1] == 0);

  // Velocity limits
  problem.subject_to(
      bounds(-maxLinearVelocity.value(), X[3, _], maxLinearVelocity.value()));
  problem.subject_to(
      bounds(-maxAngularVelocity.value(), X[4, _], maxAngularVelocity.value()));

  // Acceleration limits
  problem.subject_to(bounds(-maxLinearAcceleration.value(), U[0, _],
                            maxLinearAcceleration.value()));
  problem.subject_to(bounds(-maxAngularAcceleration.value(), U[1, _],
                            maxAngularAcceleration.value()));

  problem.minimize(std::accumulate(dts.begin(), dts.end(), slp::Variable{0.0}));

  if (auto status = problem.solve(); status != slp::ExitStatus::SUCCESS) {
    return std::unexpected{status};
  }

  std::vector<wpi::math::HolonomicSample> samples;
  auto t = 0_s;
  for (int k = 0; k < X.cols(); ++k) {
    wpi::units::meter_t x{X[0, k].value()};
    wpi::units::meter_t y{X[1, k].value()};
    wpi::units::radian_t θ{X[2, k].value()};
    wpi::units::meters_per_second_t v{X[3, k].value()};
    wpi::units::radians_per_second_t ω{X[4, k].value()};

    if (k < X.cols() - 1) {
      wpi::units::meters_per_second_squared_t a{U[0, k].value()};
      wpi::units::radians_per_second_squared_t α{U[1, k].value()};
      wpi::units::second_t dt{dts[k].value()};

      // The robot-relative acceleration includes the centripetal term vω
      wpi::units::meters_per_second_squared_t a_c{v.value() * ω.value()};

      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v, 0_mps, ω}.ToFieldRelative(θ),
          wpi::math::ChassisAccelerations{a, a_c, α}.ToFieldRelative(θ));
      t += dt;
    } else {
      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v, 0_mps, ω}.ToFieldRelative(θ),
          wpi::math::ChassisAccelerations{0_mps_sq, 0_mps_sq, 0_rad_per_s_sq}
              .ToFieldRelative(θ));
    }
  }

  return wpi::math::HolonomicTrajectory{std::move(samples)};
}

std::expected<HolonomicTrajectory, slp::ExitStatus>
UnicycleTrajectoryGenerator::Generate(
    std::span<const Pose2d> waypoints,
    wpi::units::meters_per_second_t maxLinearVelocity,
    wpi::units::radians_per_second_t maxAngularVelocity,
    wpi::units::meters_per_second_squared_t maxLinearAcceleration,
    wpi::units::radians_per_second_squared_t maxAngularAcceleration) {
  using namespace slp::slicing;

  constexpr int N_sgmt = 40;
  const int num_segments = waypoints.size() - 1;

  auto problem = slp::Problem<double>();

  auto X = problem.decision_variable(5, num_segments * N_sgmt + 1);
  auto U = problem.decision_variable(2, num_segments * N_sgmt);
  auto dts = problem.decision_variable(num_segments * N_sgmt);

  // Initial guess
  SetInitialGuess(X, dts, {waypoints.begin(), waypoints.end()}, N_sgmt,
                  maxLinearVelocity, maxAngularVelocity);

  // dt constraints
  problem.subject_to(dts >= 0);
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
      problem.subject_to(dts[k] == dts[k + 1]);
    }
  }

  // Waypoint constraints
  for (size_t sgmt = 0; sgmt < waypoints.size(); ++sgmt) {
    const auto& waypoint = waypoints[sgmt];

    auto x = X[0, sgmt * N_sgmt];
    auto y = X[1, sgmt * N_sgmt];
    auto θ = X[2, sgmt * N_sgmt];
    problem.subject_to(x == waypoint.X().value());
    problem.subject_to(y == waypoint.Y().value());
    problem.subject_to(cos(θ) == waypoint.Rotation().Cos());
    problem.subject_to(sin(θ) == waypoint.Rotation().Sin());
  }

  // Dynamics constraints
  for (int k = 0; k < X.cols() - 1; ++k) {
    auto x_k = X[_, k];
    auto x_k1 = X[_, k + 1];
    auto u_k = U[_, k];
    auto dt_k = dts[k];
    problem.subject_to(x_k1 == RK4(f, x_k, u_k, dt_k));
  }

  // Initial and final velocities
  problem.subject_to(X[slp::Slice{3, 5}, 0] == 0);
  problem.subject_to(X[slp::Slice{3, 5}, -1] == 0);

  // Velocity limits
  problem.subject_to(
      bounds(-maxLinearVelocity.value(), X[3, _], maxLinearVelocity.value()));
  problem.subject_to(
      bounds(-maxAngularVelocity.value(), X[4, _], maxAngularVelocity.value()));

  // Acceleration limits
  problem.subject_to(bounds(-maxLinearAcceleration.value(), U[0, _],
                            maxLinearAcceleration.value()));
  problem.subject_to(bounds(-maxAngularAcceleration.value(), U[1, _],
                            maxAngularAcceleration.value()));

  problem.minimize(std::accumulate(dts.begin(), dts.end(), slp::Variable{0.0}));

  if (auto status = problem.solve(); status != slp::ExitStatus::SUCCESS) {
    return std::unexpected{status};
  }

  std::vector<wpi::math::HolonomicSample> samples;
  auto t = 0_s;
  for (int k = 0; k < X.cols(); ++k) {
    wpi::units::meter_t x{X[0, k].value()};
    wpi::units::meter_t y{X[1, k].value()};
    wpi::units::radian_t θ{X[2, k].value()};
    wpi::units::meters_per_second_t v{X[3, k].value()};
    wpi::units::radians_per_second_t ω{X[4, k].value()};

    if (k < X.cols() - 1) {
      wpi::units::meters_per_second_squared_t a{U[0, k].value()};
      wpi::units::radians_per_second_squared_t α{U[1, k].value()};
      wpi::units::second_t dt{dts[k].value()};

      // The robot-relative acceleration includes the centripetal term vω
      wpi::units::meters_per_second_squared_t a_c{v.value() * ω.value()};

      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v, 0_mps, ω}.ToFieldRelative(θ),
          wpi::math::ChassisAccelerations{a, a_c, α}.ToFieldRelative(θ));
      t += dt;
    } else {
      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v, 0_mps, ω}.ToFieldRelative(θ),
          wpi::math::ChassisAccelerations{0_mps_sq, 0_mps_sq, 0_rad_per_s_sq}
              .ToFieldRelative(θ));
    }
  }

  return wpi::math::HolonomicTrajectory{std::move(samples)};
}

}  // namespace wpi::math
