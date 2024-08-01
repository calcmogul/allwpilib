// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/proto/TrajectoryStateProto.h"

#include <utility>

#include <wpi/protobuf/ProtobufCallbacks.h>

#include "wpimath/protobuf/trajectory.npb.h"

std::optional<frc::Trajectory::State>
wpi::Protobuf<frc::Trajectory::State>::Unpack(InputStream& stream) {
  wpi::UnpackCallback<frc::Pose2d> pose;
  wpi_proto_ProtobufTrajectoryState msg;
  msg.pose = pose.Callback();

  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  if (ipose.empty()) {
    return {};
  }

  return frc::Trajectory::State{
      units::second_t{msg.time},
      std::move(ipose[0]),
      units::meters_per_second_t{msg.linearVelocity},
      units::meters_per_second_squared_t{msg.linearAcceleration},
      units::radians_per_second_t{msg.angularVelocity},
  };
}

bool wpi::Protobuf<frc::Trajectory::State>::Pack(
    OutputStream& stream, const frc::Trajectory::State& value) {
  wpi::PackCallback pose{&value.pose};
  wpi_proto_ProtobufTrajectoryState msg{
      .time = value.t.value(),
      .pose = pose.Callback(),
      .linearVelocity = value.linearVelocity.value(),
      .linearAcceleration = value.linearAcceleration.value(),
      .angularVelocity = value.angularVelocity.value(),
  };
  return stream.Encode(msg);
}
