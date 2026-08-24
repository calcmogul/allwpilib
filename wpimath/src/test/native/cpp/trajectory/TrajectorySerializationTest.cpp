// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstddef>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "wpi/math/kinematics/DifferentialDriveKinematics.hpp"
#include "wpi/math/trajectory/DifferentialSample.hpp"
#include "wpi/math/trajectory/DifferentialTrajectory.hpp"
#include "wpi/math/trajectory/HolonomicSample.hpp"
#include "wpi/math/trajectory/HolonomicTrajectory.hpp"
#include "wpi/math/trajectory/TestHolonomicTrajectory.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/velocity.hpp"
#include "wpi/util/json.hpp"

using namespace wpi::math;

TEST_CASE("TrajectorySerializationTest TestJsonSerialization", "[wpimath]") {
  auto trajectory = TestHolonomicTrajectory::GetTrajectory(12_fps, 12_fps_sq);

  // Serialize to JSON
  wpi::util::json json;
  to_json(json, trajectory);

  // Deserialize from JSON
  auto samples_out = json.at("samples").get<std::vector<HolonomicSample>>();
  HolonomicTrajectory deserializedTrajectory{std::move(samples_out)};

  // Verify they are equal
  CHECK(trajectory.Duration() == deserializedTrajectory.Duration());
  CHECK(trajectory.Samples().size() == deserializedTrajectory.Samples().size());

  for (size_t i = 0; i < trajectory.Samples().size(); ++i) {
    const auto& original = trajectory.Samples()[i];
    const auto& deserialized = deserializedTrajectory.Samples()[i];

    CHECK(original.time == deserialized.time);
    CHECK(original.pose == deserialized.pose);
    CHECK(original.velocity == deserialized.velocity);
    CHECK(original.acceleration == deserialized.acceleration);
  }
}

TEST_CASE("TrajectorySerializationTest TestDifferentialSerialization",
          "[wpimath]") {
  auto holonomicTrajectory =
      TestHolonomicTrajectory::GetTrajectory(12_fps, 12_fps_sq);

  DifferentialDriveKinematics kinematics{0.5_m};
  std::vector<DifferentialSample> samples;
  for (const auto& holonomicSample : holonomicTrajectory.Samples()) {
    samples.emplace_back(holonomicSample, kinematics);
  }

  DifferentialTrajectory trajectory{std::move(samples)};

  // Serialize to JSON
  wpi::util::json json;
  to_json(json, trajectory);

  // Deserialize from JSON
  auto samples_out = json.at("samples").get<std::vector<DifferentialSample>>();
  DifferentialTrajectory deserializedTrajectory{std::move(samples_out)};

  // Verify they are equal
  CHECK(trajectory.Duration() == deserializedTrajectory.Duration());
  CHECK(trajectory.Samples().size() == deserializedTrajectory.Samples().size());

  for (size_t i = 0; i < trajectory.Samples().size(); ++i) {
    const auto& original = trajectory.Samples()[i];
    const auto& deserialized = deserializedTrajectory.Samples()[i];

    CHECK(original.time == deserialized.time);
    CHECK(original.pose == deserialized.pose);
    CHECK(original.velocity == deserialized.velocity);
    CHECK(original.acceleration == deserialized.acceleration);
    CHECK(original.leftVelocity == deserialized.leftVelocity);
    CHECK(original.rightVelocity == deserialized.rightVelocity);
  }
}
