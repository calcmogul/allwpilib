// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/TrajectoryParameterizer.h"

#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <sleipnir/optimization/OptimizationProblem.hpp>
#include <sleipnir/optimization/SolverExitCondition.hpp>

using namespace frc;

Trajectory TrajectoryParameterizer::TimeParameterizeTrajectory(
    const std::vector<PoseWithCurvature>& points,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity,
    units::meters_per_second_t maxVelocity,
    units::meters_per_second_squared_t maxAcceleration, bool reversed) {
  sleipnir::OptimizationProblem problem;

  auto s = problem.DecisionVariable(1, points.size());
  auto v = problem.DecisionVariable(1, points.size());
  auto a = problem.DecisionVariable(1, points.size() - 1);
  auto dt = problem.DecisionVariable(1, points.size() - 1);

  // Global max linear velocity
  problem.SubjectTo(v >= -maxVelocity.value());
  problem.SubjectTo(v <= maxVelocity.value());

  // Global max acceleration
  problem.SubjectTo(a >= -maxAcceleration.value());
  problem.SubjectTo(a <= maxAcceleration.value());

  units::meter_t distance = 0_m;
  for (size_t k = 0; k < points.size() - 1; ++k) {
    const auto& pose_k = points[k].first;
    const auto& pose_k1 = points[k + 1].first;
    auto twist_k = pose_k.Log(pose_k1);
    auto ds = units::math::hypot(twist_k.dx, twist_k.dy);

    problem.SubjectTo(s(k) == distance.value());
    s(k).SetValue(distance.value());

    distance += ds;

    problem.SubjectTo(dt(k) >= 0.0);
    dt(k).SetValue(ds.value() / maxVelocity.value());
  }
  s(points.size() - 1).SetValue(distance.value());

  for (size_t k = 0; k < points.size(); ++k) {
    const auto& pose_k = points[k].first;

    double curvature = points[k].second.value();

    // ωₖ = vₖκₖ
    auto ω_k = v(k) * curvature;

    if (k < points.size() - 1) {
      // sₖ₊₁ = sₖ + vₖΔtₖ + 1/2aₖΔtₖ²
      problem.SubjectTo(s(k + 1) ==
                        s(k) + v(k) * dt(k) + 0.5 * a(k) * dt(k) * dt(k));

      // vₖ₊₁ = vₖ + aₖΔtₖ
      problem.SubjectTo(v(k + 1) == v(k) + a(k) * dt(k));

      // Enforce user constraints
      for (auto&& constraint : constraints) {
        // αₖ = aₖκₖ
        auto α_k = a(k) * curvature;

        constraint->Apply(problem, pose_k, v(k), ω_k, a(k), α_k);
      }
    }
  }

  // Initial velocity
  problem.SubjectTo(v(0) == startVelocity.value());
  v(0).SetValue(startVelocity.value());

  // Final velocity
  problem.SubjectTo(v(points.size() - 1) == endVelocity.value());
  v(points.size() - 1).SetValue(endVelocity.value());

  // Minimize time
  problem.Minimize(
      std::accumulate(dt.begin(), dt.end(), sleipnir::Variable{0.0}));

  auto status = problem.Solve({.diagnostics = true});
  if (static_cast<int>(status.exitCondition) < 0) {
    auto msg = std::string{sleipnir::ToMessage(status.exitCondition)};
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
    double curvature = points[k].second.value();
    auto ω_k = v(k).Value() * curvature;

    if (k < points.size() - 1) {
      states.emplace_back(
          t, points[k].first,
          units::meters_per_second_t{reversed ? -v(k).Value() : v(k).Value()},
          units::meters_per_second_squared_t{reversed ? -a(k).Value()
                                                      : a(k).Value()},
          units::radians_per_second_t{reversed ? -ω_k : ω_k});
      t += units::second_t{dt(k).Value()};
    } else {
      states.emplace_back(
          t, points[k].first,
          units::meters_per_second_t{reversed ? -v(k).Value() : v(k).Value()},
          0_mps_sq, units::radians_per_second_t{reversed ? -ω_k : ω_k});
    }
  }

  return Trajectory{states};
}
