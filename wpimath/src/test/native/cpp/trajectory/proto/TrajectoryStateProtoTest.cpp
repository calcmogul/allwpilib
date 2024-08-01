// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/trajectory/Trajectory.h"

using namespace frc;

namespace {

using ProtoType = wpi::Protobuf<frc::Trajectory::State>;

const Trajectory::State kExpectedData = Trajectory::State{
    1.91_s, Pose2d{Translation2d{1.74_m, 19.1_m}, Rotation2d{22.9_rad}},
    4.4_mps, 17.4_mps_sq, 0.174_rad_per_s};

}  // namespace

TEST(TrajectoryStateProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(kExpectedData.t.value(), unpacked_data->t.value());
  EXPECT_EQ(kExpectedData.pose, unpacked_data->pose);
  EXPECT_EQ(kExpectedData.linearVelocity.value(),
            unpacked_data->linearVelocity.value());
  EXPECT_EQ(kExpectedData.linearAcceleration.value(),
            unpacked_data->linearAcceleration.value());
  EXPECT_EQ(kExpectedData.angularVelocity.value(),
            unpacked_data->angularVelocity.value());
}
