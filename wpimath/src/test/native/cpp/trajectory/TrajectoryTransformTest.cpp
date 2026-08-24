// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "wpi/math/TestAssertions.hpp"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/trajectory/HolonomicSample.hpp"
#include "wpi/math/trajectory/HolonomicTrajectoryGenerator.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/units/velocity.hpp"

void TestSameShapedTrajectory(
    const std::vector<wpi::math::HolonomicSample>& statesA,
    const std::vector<wpi::math::HolonomicSample>& statesB) {
  for (unsigned int i = 0; i < statesA.size() - 1; i++) {
    auto a1 = statesA[i].pose;
    auto a2 = statesA[i + 1].pose;

    auto b1 = statesB[i].pose;
    auto b2 = statesB[i + 1].pose;

    auto a = a2.RelativeTo(a1);
    auto b = b2.RelativeTo(b1);

    CHECK_NEAR(a.X().value(), b.X().value(), 1E-9);
    CHECK_NEAR(a.Y().value(), b.Y().value(), 1E-9);
    CHECK_NEAR(a.Rotation().Radians().value(), b.Rotation().Radians().value(),
               1E-9);
  }
}

// A rigid transform rotates both the heading and the field-relative
// velocity/acceleration by the same amount, so the heading-relative forward
// scalars (and curvature) are invariant. This would fail if
// TransformBy/RelativeTo rotated the pose but not the velocity/acceleration.
void TestSameForwardScalars(
    const std::vector<wpi::math::HolonomicSample>& statesA,
    const std::vector<wpi::math::HolonomicSample>& statesB) {
  REQUIRE(statesA.size() == statesB.size());
  for (unsigned int i = 0; i < statesA.size(); i++) {
    CHECK_NEAR(statesA[i].velocity.vx.value(), statesB[i].velocity.vx.value(),
               1E-9);
    CHECK_NEAR(statesA[i].velocity.vy.value(), statesB[i].velocity.vy.value(),
               1E-9);
    CHECK_NEAR(statesA[i].velocity.omega.value(),
               statesB[i].velocity.omega.value(), 1E-9);
    CHECK_NEAR(statesA[i].acceleration.ax.value(),
               statesB[i].acceleration.ax.value(), 1E-9);
    CHECK_NEAR(statesA[i].acceleration.ay.value(),
               statesB[i].acceleration.ay.value(), 1E-9);
    CHECK_NEAR(statesA[i].acceleration.alpha.value(),
               statesB[i].acceleration.alpha.value(), 1E-9);
  }
}

TEST_CASE("TrajectoryTransformsTest TransformBy", "[wpimath]") {
  auto trajectory = wpi::math::HolonomicTrajectoryGenerator::Generate(
      wpi::math::Pose2d{}, {}, wpi::math::Pose2d{1_m, 1_m, 90_deg}, 3_mps,
      1_rad_per_s, 3_mps_sq, 1_rad_per_s_sq);

  auto transformedTrajectory = trajectory.TransformBy({{1_m, 2_m}, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 1.0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 2.0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 30.0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameForwardScalars(trajectory.Samples(), transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformsTest RelativeTo", "[wpimath]") {
  auto trajectory = wpi::math::HolonomicTrajectoryGenerator::Generate(
      wpi::math::Pose2d{1_m, 2_m, 30_deg}, {},
      wpi::math::Pose2d{5_m, 7_m, 90_deg}, 3_mps, 1_rad_per_s, 3_mps_sq,
      1_rad_per_s_sq);

  auto transformedTrajectory = trajectory.RelativeTo({1_m, 2_m, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameForwardScalars(trajectory.Samples(), transformedTrajectory.Samples());
}
