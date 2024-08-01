// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/proto/TrajectoryStateProto.h"

#include <wpi/ProtoHelper.h>

#include "trajectory.pb.h"

google::protobuf::Message* wpi::Protobuf<frc::Trajectory::State>::New(
    google::protobuf::Arena* arena) {
  return wpi::CreateMessage<wpi::proto::ProtobufTrajectoryState>(arena);
}

frc::Trajectory::State wpi::Protobuf<frc::Trajectory::State>::Unpack(
    const google::protobuf::Message& msg) {
  auto m = static_cast<const wpi::proto::ProtobufTrajectoryState*>(&msg);
  return frc::Trajectory::State{
      units::second_t{m->time()},
      wpi::UnpackProtobuf<frc::Pose2d>(m->pose()),
      units::meters_per_second_t{m->linearvelocity()},
      units::meters_per_second_squared_t{m->linearacceleration()},
      units::radians_per_second_t{m->angularvelocity()},
  };
}

void wpi::Protobuf<frc::Trajectory::State>::Pack(
    google::protobuf::Message* msg, const frc::Trajectory::State& value) {
  auto m = static_cast<wpi::proto::ProtobufTrajectoryState*>(msg);
  m->set_time(value.t.value());
  wpi::PackProtobuf(m->mutable_pose(), value.pose);
  m->set_linearvelocity(value.linearVelocity.value());
  m->set_linearacceleration(value.linearAcceleration.value());
  m->set_angularvelocity(value.angularVelocity.value());
}
