// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/Trajectory.h"

#include <wpi/json.h>

using namespace frc;

void frc::to_json(wpi::json& json, const Trajectory::State& state) {
  json = wpi::json{{"time", state.t.value()},
                   {"pose", state.pose},
                   {"linearVelocity", state.linearVelocity.value()},
                   {"linearAcceleration", state.linearAcceleration.value()},
                   {"angularVelocity", state.angularVelocity.value()}};
}

void frc::from_json(const wpi::json& json, Trajectory::State& state) {
  state.t = units::second_t{json.at("time").get<double>()};
  state.pose = json.at("pose").get<Pose2d>();
  state.linearVelocity =
      units::meters_per_second_t{json.at("linearVelocity").get<double>()};
  state.linearAcceleration = units::meters_per_second_squared_t{
      json.at("linearAcceleration").get<double>()};
  state.angularVelocity =
      units::radians_per_second_t{json.at("angularVelocity").get<double>()};
}
