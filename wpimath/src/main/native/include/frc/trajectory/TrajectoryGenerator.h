// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <wpi/SymbolExports.h>
#include <wpi/function.h>
#include <wpi/print.h>

#include "frc/spline/SplineHelper.h"
#include "frc/spline/SplineParameterizer.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/TrajectoryParameterizer.h"
#include "frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {

/**
 * Helper class used to generate trajectories with various constraints.
 */
class WPILIB_DLLEXPORT TrajectoryGenerator {
 public:
  using PoseWithCurvature = std::pair<Pose2d, units::curvature_t>;

  /**
   * Generates a trajectory from the given control vectors and config. This
   * method uses clamped cubic splines -- a method in which the exterior control
   * vectors and interior waypoints are provided. The headings are automatically
   * determined at the interior points to ensure continuous curvature.
   *
   * @param initial           The initial control vector.
   * @param interiorWaypoints The interior waypoints.
   * @param end               The ending control vector.
   * @param config            The configuration for the trajectory.
   * @return The generated trajectory.
   */
  static constexpr std::optional<Trajectory> GenerateTrajectory(
      Spline<3>::ControlVector initial,
      const std::vector<Translation2d>& interiorWaypoints,
      Spline<3>::ControlVector end, const TrajectoryConfig& config) {
    const Transform2d flip{Translation2d{}, 180_deg};

    // Make theta normal for trajectory generation if path is reversed.
    // Flip the headings.
    if (config.IsReversed()) {
      initial.x[1] *= -1;
      initial.y[1] *= -1;
      end.x[1] *= -1;
      end.y[1] *= -1;
    }

    std::vector<frc::SplineParameterizer::PoseWithCurvature> points;
    try {
      points =
          SplinePointsFromSplines(SplineHelper::CubicSplinesFromControlVectors(
              initial, interiorWaypoints, end));
    } catch (SplineParameterizer::MalformedSplineException& e) {
      return std::nullopt;
    }

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (config.IsReversed()) {
      for (auto& point : points) {
        point = {point.first + flip, -point.second};
      }
    }

    return TrajectoryParameterizer::TimeParameterizeTrajectory(
        points, config.Constraints(), config.StartVelocity(),
        config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
        config.IsReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method
   * uses clamped cubic splines -- a method in which the initial pose, final
   * pose, and interior waypoints are provided.  The headings are automatically
   * determined at the interior points to ensure continuous curvature.
   *
   * @param start             The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end               The ending pose.
   * @param config            The configuration for the trajectory.
   * @return The generated trajectory.
   */
  static constexpr std::optional<Trajectory> GenerateTrajectory(
      const Pose2d& start, const std::vector<Translation2d>& interiorWaypoints,
      const Pose2d& end, const TrajectoryConfig& config) {
    auto [startCV, endCV] = SplineHelper::CubicControlVectorsFromWaypoints(
        start, interiorWaypoints, end);
    return GenerateTrajectory(startCV, interiorWaypoints, endCV, config);
  }

  /**
   * Generates a trajectory from the given quintic control vectors and config.
   * This method uses quintic hermite splines -- therefore, all points must be
   * represented by control vectors. Continuous curvature is guaranteed in this
   * method.
   *
   * @param controlVectors List of quintic control vectors.
   * @param config         The configuration for the trajectory.
   * @return The generated trajectory.
   */
  static constexpr std::optional<Trajectory> GenerateTrajectory(
      std::vector<Spline<5>::ControlVector> controlVectors,
      const TrajectoryConfig& config) {
    const Transform2d flip{Translation2d{}, 180_deg};
    // Make theta normal for trajectory generation if path is reversed.
    if (config.IsReversed()) {
      for (auto& vector : controlVectors) {
        // Flip the headings.
        vector.x[1] *= -1;
        vector.y[1] *= -1;
      }
    }

    std::vector<frc::SplineParameterizer::PoseWithCurvature> points;
    try {
      points = SplinePointsFromSplines(
          SplineHelper::QuinticSplinesFromControlVectors(controlVectors));
    } catch (SplineParameterizer::MalformedSplineException& e) {
      return std::nullopt;
    }

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (config.IsReversed()) {
      for (auto& point : points) {
        point = {point.first + flip, -point.second};
      }
    }

    return TrajectoryParameterizer::TimeParameterizeTrajectory(
        points, config.Constraints(), config.StartVelocity(),
        config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
        config.IsReversed());
  }

  /**
   * Generates a trajectory from the given waypoints and config. This method
   * uses quintic hermite splines -- therefore, all points must be represented
   * by Pose2d objects. Continuous curvature is guaranteed in this method.
   *
   * @param waypoints List of waypoints..
   * @param config    The configuration for the trajectory.
   * @return The generated trajectory.
   */
  static constexpr std::optional<Trajectory> GenerateTrajectory(
      const std::vector<Pose2d>& waypoints, const TrajectoryConfig& config) {
    auto newWaypoints = waypoints;
    const Transform2d flip{Translation2d{}, 180_deg};
    if (config.IsReversed()) {
      for (auto& waypoint : newWaypoints) {
        waypoint = waypoint + flip;
      }
    }

    std::vector<SplineParameterizer::PoseWithCurvature> points;
    try {
      points = SplinePointsFromSplines(SplineHelper::OptimizeCurvature(
          SplineHelper::QuinticSplinesFromWaypoints(newWaypoints)));
    } catch (SplineParameterizer::MalformedSplineException& e) {
      return std::nullopt;
    }

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (config.IsReversed()) {
      for (auto& point : points) {
        point = {point.first + flip, -point.second};
      }
    }

    return TrajectoryParameterizer::TimeParameterizeTrajectory(
        points, config.Constraints(), config.StartVelocity(),
        config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
        config.IsReversed());
  }

  /**
   * Generate spline points from a vector of splines by parameterizing the
   * splines.
   *
   * @param splines The splines to parameterize.
   *
   * @return The spline points for use in time parameterization of a trajectory.
   */
  template <typename Spline>
  static constexpr std::vector<PoseWithCurvature> SplinePointsFromSplines(
      const std::vector<Spline>& splines) {
    // Create the vector of spline points.
    std::vector<PoseWithCurvature> splinePoints;

    // Add the first point to the vector.
    splinePoints.push_back(splines.front().GetPoint(0.0).value());

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    for (auto&& spline : splines) {
      auto points = SplineParameterizer::Parameterize(spline);
      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.insert(std::end(splinePoints), std::begin(points) + 1,
                          std::end(points));
    }
    return splinePoints;
  }
};

}  // namespace frc
