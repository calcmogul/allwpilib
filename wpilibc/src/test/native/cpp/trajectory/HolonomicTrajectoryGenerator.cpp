// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/trajectory/HolonomicTrajectoryGenerator.hpp"

#include <cmath>
#include <cstddef>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include <sleipnir/autodiff/slice.hpp>
#include <sleipnir/autodiff/variable.hpp>
#include <sleipnir/autodiff/variable_matrix.hpp>
#include <sleipnir/optimization/problem.hpp>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/kinematics/ChassisAccelerations.hpp"
#include "wpi/math/kinematics/ChassisVelocities.hpp"
#include "wpi/math/trajectory/HolonomicSample.hpp"
#include "wpi/math/trajectory/HolonomicTrajectory.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
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

}  // namespace

namespace wpi::math {

HolonomicTrajectory HolonomicTrajectoryGenerator::Generate(
    const Pose2d& initialWaypoint,
    std::span<const Translation2d> interiorWaypoints,
    const Pose2d& finalWaypoint,
    wpi::units::meters_per_second_t maxLinearVelocity,
    wpi::units::radians_per_second_t maxAngularVelocity,
    wpi::units::meters_per_second_squared_t maxLinearAcceleration,
    wpi::units::radians_per_second_squared_t maxAngularAcceleration) {
  using namespace slp::slicing;

  auto f = [](const slp::VariableMatrix<double>& x,
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
  };

  constexpr int N_sgmt = 40;
  const int num_segments = interiorWaypoints.size() + 1;
  constexpr double T_max = 5.0;

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

  // Pose initial guess
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1); ++k) {
      double scale = static_cast<double>(k - N_sgmt * sgmt) / N_sgmt;
      X[0, k].set_value(std::lerp(waypoints[sgmt].X().value(),
                                  waypoints[sgmt + 1].X().value(), scale));
      X[1, k].set_value(std::lerp(waypoints[sgmt].Y().value(),
                                  waypoints[sgmt + 1].Y().value(), scale));
      X[2, k].set_value((waypoints[sgmt + 1] - waypoints[sgmt])
                            .Angle()
                            .value_or(0_rad)
                            .Radians()
                            .value());
    }
  }
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    X[0, N_sgmt * sgmt].set_value(waypoints[sgmt].X().value());
    X[1, N_sgmt * sgmt].set_value(waypoints[sgmt].Y().value());
    if (sgmt == 0) {
      X[2, N_sgmt * sgmt].set_value(
          initialWaypoint.Rotation().Radians().value());
    } else if (sgmt == num_segments - 1) {
      X[2, N_sgmt * sgmt].set_value(finalWaypoint.Rotation().Radians().value());
    }
  }
  X[0, X.cols() - 1].set_value(finalWaypoint.X().value());
  X[1, X.cols() - 1].set_value(finalWaypoint.Y().value());
  X[2, X.cols() - 1].set_value(finalWaypoint.Rotation().Radians().value());

  // dt constraints
  problem.subject_to(bounds(0, dts, T_max / (num_segments * N_sgmt)));
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
      problem.subject_to(dts[k] == dts[k + 1]);
    }
  }
  for (int k = 0; k < X.cols() - 1; ++k) {
    dts[k].set_value(T_max / N_sgmt);
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

  problem.solve();

  std::vector<wpi::math::HolonomicSample> samples;
  auto t = 0_s;
  for (int k = 0; k < X.cols(); ++k) {
    wpi::units::meter_t x{X[0, k].value()};
    wpi::units::meter_t y{X[1, k].value()};
    wpi::units::radian_t θ{X[2, k].value()};
    wpi::units::meters_per_second_t v_x{X[3, k].value()};
    wpi::units::radians_per_second_t ω{X[4, k].value()};

    if (k < X.cols() - 1) {
      wpi::units::meters_per_second_squared_t a_x{U[0, k].value()};
      wpi::units::radians_per_second_squared_t α{U[1, k].value()};
      wpi::units::second_t dt{dts[k].value()};
      samples.emplace_back(t, wpi::math::Pose2d{x, y, θ},
                           wpi::math::ChassisVelocities{v_x, 0_mps, ω},
                           wpi::math::ChassisAccelerations{a_x, 0_mps_sq, α});
      t += dt;
    } else {
      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v_x, 0_mps, ω},
          wpi::math::ChassisAccelerations{0_mps_sq, 0_mps_sq, 0_rad_per_s_sq});
    }
  }

  return wpi::math::HolonomicTrajectory{std::move(samples)};
}

HolonomicTrajectory HolonomicTrajectoryGenerator::Generate(
    std::span<const Pose2d> waypoints,
    wpi::units::meters_per_second_t maxLinearVelocity,
    wpi::units::radians_per_second_t maxAngularVelocity,
    wpi::units::meters_per_second_squared_t maxLinearAcceleration,
    wpi::units::radians_per_second_squared_t maxAngularAcceleration) {
  using namespace slp::slicing;

  auto f = [](const slp::VariableMatrix<double>& x,
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
  };

  constexpr int N_sgmt = 40;
  const int num_segments = waypoints.size() - 1;
  constexpr double T_max = 5.0;

  auto problem = slp::Problem<double>();

  auto X = problem.decision_variable(5, num_segments * N_sgmt + 1);
  auto U = problem.decision_variable(2, num_segments * N_sgmt);
  auto dts = problem.decision_variable(num_segments * N_sgmt);

  // Pose initial guess
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1); ++k) {
      double scale = static_cast<double>(k - N_sgmt * sgmt) / N_sgmt;
      X[0, k].set_value(std::lerp(waypoints[sgmt].X().value(),
                                  waypoints[sgmt + 1].X().value(), scale));
      X[1, k].set_value(std::lerp(waypoints[sgmt].Y().value(),
                                  waypoints[sgmt + 1].Y().value(), scale));
      X[2, k].set_value(
          (waypoints[sgmt + 1].Translation() - waypoints[sgmt].Translation())
              .Angle()
              .value_or(0_rad)
              .Radians()
              .value());
    }
  }
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    X[0, N_sgmt * sgmt].set_value(waypoints[sgmt].X().value());
    X[1, N_sgmt * sgmt].set_value(waypoints[sgmt].Y().value());
    X[2, N_sgmt * sgmt].set_value(waypoints[sgmt].Rotation().Radians().value());
  }
  X[0, X.cols() - 1].set_value(waypoints[num_segments - 1].X().value());
  X[1, X.cols() - 1].set_value(waypoints[num_segments - 1].Y().value());
  X[2, X.cols() - 1].set_value(
      waypoints[num_segments - 1].Rotation().Radians().value());

  // dt constraints
  problem.subject_to(bounds(0, dts, T_max / (num_segments * N_sgmt)));
  for (int sgmt = 0; sgmt < num_segments; ++sgmt) {
    for (int k = N_sgmt * sgmt; k < N_sgmt * (sgmt + 1) - 1; ++k) {
      problem.subject_to(dts[k] == dts[k + 1]);
    }
  }
  for (int k = 0; k < X.cols() - 1; ++k) {
    dts[k].set_value(T_max / N_sgmt);
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

  problem.solve();

  std::vector<wpi::math::HolonomicSample> samples;
  auto t = 0_s;
  for (int k = 0; k < X.cols(); ++k) {
    wpi::units::meter_t x{X[0, k].value()};
    wpi::units::meter_t y{X[1, k].value()};
    wpi::units::radian_t θ{X[2, k].value()};
    wpi::units::meters_per_second_t v_x{X[3, k].value()};
    wpi::units::radians_per_second_t ω{X[4, k].value()};
    wpi::units::meters_per_second_squared_t a_x{U[0, k].value()};
    wpi::units::radians_per_second_squared_t α{U[1, k].value()};
    wpi::units::second_t dt{dts[k].value()};

    if (k < X.cols() - 1) {
      samples.emplace_back(t, wpi::math::Pose2d{x, y, θ},
                           wpi::math::ChassisVelocities{v_x, 0_mps, ω},
                           wpi::math::ChassisAccelerations{a_x, 0_mps_sq, α});
      t += dt;
    } else {
      samples.emplace_back(
          t, wpi::math::Pose2d{x, y, θ},
          wpi::math::ChassisVelocities{v_x, 0_mps, ω},
          wpi::math::ChassisAccelerations{0_mps_sq, 0_mps_sq, 0_rad_per_s_sq});
    }
  }

  return wpi::math::HolonomicTrajectory{std::move(samples)};
}

}  // namespace wpi::math
