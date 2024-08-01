// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/TrajectoryParameterizer.h"

#include <numeric>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <sleipnir/optimization/problem.hpp>
#include <sleipnir/optimization/solver/exit_status.hpp>

#include "frc/trajectory/Trajectory.h"

using namespace frc;

Trajectory TrajectoryParameterizer::TimeParameterizeTrajectory(
    const std::vector<PoseWithCurvature>& points,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity,
    units::meters_per_second_t maxVelocity,
    units::meters_per_second_squared_t maxAcceleration, bool reversed) {
  slp::Problem problem;

  auto s = problem.decision_variable(1, points.size());
  auto v = problem.decision_variable(1, points.size());
  auto a = problem.decision_variable(1, points.size() - 1);
  auto dt = problem.decision_variable(1, points.size() - 1);

  // Global max linear velocity
  problem.subject_to(v >= -maxVelocity.value());
  problem.subject_to(v <= maxVelocity.value());

  // Global max acceleration
  problem.subject_to(a >= -maxAcceleration.value());
  problem.subject_to(a <= maxAcceleration.value());

  units::meter_t distance = 0_m;
  for (size_t k = 0; k < points.size() - 1; ++k) {
    const auto& pose_k = points[k].first;
    const auto& pose_k1 = points[k + 1].first;
    auto twist_k = (pose_k1 - pose_k).Log();
    auto ds = units::math::hypot(twist_k.dx, twist_k.dy);

    problem.subject_to(s[k] == distance.value());
    s[k].set_value(distance.value());

    distance += ds;

    problem.subject_to(dt[k] >= 0.0);
    dt[k].set_value(ds.value() / maxVelocity.value());
  }
  s[points.size() - 1].set_value(distance.value());

  for (size_t k = 0; k < points.size(); ++k) {
    const auto& pose_k = points[k].first;

    double curvature = points[k].second.value();

    // ωₖ = vₖκₖ
    auto ω_k = v[k] * curvature;

    if (k < points.size() - 1) {
      // sₖ₊₁ = sₖ + vₖΔtₖ + 1/2aₖΔtₖ²
      problem.subject_to(s[k + 1] ==
                         s[k] + v[k] * dt[k] + 0.5 * a[k] * dt[k] * dt[k]);

      // vₖ₊₁ = vₖ + aₖΔtₖ
      problem.subject_to(v[k + 1] == v[k] + a[k] * dt[k]);

      // Enforce user constraints
      for (auto&& constraint : constraints) {
        // αₖ = aₖκₖ
        auto α_k = a[k] * curvature;

        constraint->Apply(problem, pose_k, v[k], ω_k, a[k], α_k);
      }
    }
  }

  // Initial velocity
  problem.subject_to(v[0] == startVelocity.value());
  v[0].set_value(startVelocity.value());

  // Final velocity
  problem.subject_to(v[points.size() - 1] == endVelocity.value());
  v[points.size() - 1].set_value(endVelocity.value());

  // Minimize time
  problem.minimize(std::accumulate(dt.begin(), dt.end(), slp::Variable{0.0}));

  auto status = problem.solve({.diagnostics = true});
  if (static_cast<int>(status) < 0) {
    auto msg = std::string{slp::to_message(status)};
    msg[0] = std::toupper(msg[0]);
    throw std::runtime_error(fmt::format(
        "{}. If the trajectory was infeasible, determine which one by removing "
        "them all from the TrajectoryConfig and adding them back one-by-one.",
        msg));
  }

  // Now we can integrate the constrained states forward in time to obtain our
  // trajectory states.
  std::vector<Trajectory::State> states;
  states.reserve(points.size());

  units::second_t t = 0_s;
  for (size_t k = 0; k < points.size(); ++k) {
    // ωₖ = vₖκₖ
    units::meters_per_second_t linearVelocity{v[k].value()};
    auto curvature = points[k].second;
    auto ω_k = linearVelocity * curvature;

    units::meters_per_second_squared_t linearAcceleration{
        k < points.size() - 1 ? a[k].value() : 0.0};

    if (reversed) {
      states.emplace_back(t, points[k].first, -linearVelocity,
                          -linearAcceleration, -ω_k);
    } else {
      states.emplace_back(t, points[k].first, linearVelocity,
                          linearAcceleration, ω_k);
    }
  }

  return Trajectory{states};
}
