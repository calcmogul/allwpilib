// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/trajectory/Trajectory.h"

using namespace frc;

namespace {

using ProtoType = wpi::Protobuf<frc::Trajectory>;

const Trajectory kExpectedData = Trajectory{std::vector<frc::Trajectory::State>{
    Trajectory::State{1.1_s,
                      Pose2d(Translation2d(1.1_m, 2.2_m), Rotation2d(2.2_rad)),
                      2.2_mps, 3.3_mps_sq, 6.6_rad_per_s},
    Trajectory::State{2.1_s,
                      Pose2d(Translation2d(2.1_m, 2.2_m), Rotation2d(2.2_rad)),
                      2.2_mps, 3.3_mps_sq, 6.6_rad_per_s},
    Trajectory::State{3.1_s,
                      Pose2d(Translation2d(3.1_m, 2.2_m), Rotation2d(2.2_rad)),
                      2.2_mps, 3.3_mps_sq, 6.6_rad_per_s}}};
}  // namespace

TEST(TrajectoryProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());
  EXPECT_EQ(kExpectedData.States(), unpacked_data->States());
}
