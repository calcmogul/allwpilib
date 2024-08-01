// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <gtest/gtest.h>

#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(TrajectoryGeneratorTest, ObeysConstraints) {
  constexpr units::meters_per_second_t maxVelocity = 12_fps;
  constexpr units::meters_per_second_squared_t maxAcceleration = 12_fps_sq;

  TrajectoryConfig config{maxVelocity, maxAcceleration};
  auto trajectory = TestTrajectory::GetTrajectory(config);

  units::second_t time = 0_s;
  units::second_t dt = 20_ms;
  units::second_t duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    EXPECT_GE(point.linearVelocity.value(), -maxVelocity.value());
    EXPECT_LE(point.linearVelocity.value(), maxVelocity.value());

    EXPECT_GE(point.linearAcceleration.value(), -maxAcceleration.value());
    EXPECT_LE(point.linearAcceleration.value(), maxAcceleration.value());
  }
}

TEST(TrajectoryGeneratorTest, ReturnsEmptyOnMalformed) {
  const auto t = TrajectoryGenerator::GenerateTrajectory(
      std::vector<Pose2d>{Pose2d{0_m, 0_m, 0_deg}, Pose2d{1_m, 0_m, 180_deg}},
      TrajectoryConfig(12_fps, 12_fps_sq));

  ASSERT_EQ(t.States().size(), 1u);
  ASSERT_EQ(t.TotalTime(), 0_s);
}

TEST(TrajectoryGeneratorTest, CurvatureOptimization) {
  auto t = TrajectoryGenerator::GenerateTrajectory(
      {{1_m, 0_m, 90_deg},
       {0_m, 1_m, 180_deg},
       {-1_m, 0_m, 270_deg},
       {0_m, -1_m, 0_deg},
       {1_m, 0_m, 90_deg}},
      TrajectoryConfig{12_fps, 12_fps_sq});

  for (size_t i = 1; i < t.States().size() - 1; ++i) {
    EXPECT_NE(
        0,
        (t.States()[i].linearVelocity * t.States()[i].angularVelocity).value());
  }
}
