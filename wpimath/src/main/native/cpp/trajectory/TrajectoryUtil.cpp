// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/TrajectoryUtil.h"

#include <fstream>
#include <string>
#include <system_error>

#include <fmt/format.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>

using namespace frc;

void TrajectoryUtil::ToPathweaverJson(const Trajectory& trajectory,
                                      std::string_view path) {
  std::ofstream output{std::string{path}};
  if (!output.is_open()) {
    throw std::runtime_error(fmt::format("Cannot open file: {}", path));
  }

  wpi::json json = trajectory.States();
  output << json;
  output.flush();
}

Trajectory TrajectoryUtil::FromPathweaverJson(std::string_view path) {
  auto fileBuffer = wpi::MemoryBuffer::GetFile(path);
  if (!fileBuffer) {
    throw std::runtime_error(fmt::format("Cannot open file: {}", path));
  }

  wpi::json json = wpi::json::parse(fileBuffer.value()->GetCharBuffer());

  wpi::math::MathSharedStore::ReportUsage(
      wpi::math::MathUsageId::kTrajectory_PathWeaver,
      ++pathWeaverTrajectoryInstances);

  return Trajectory{json.get<std::vector<Trajectory::State>>()};
}

std::string TrajectoryUtil::SerializeTrajectory(const Trajectory& trajectory) {
  wpi::json json = trajectory.States();
  return json.dump();
}

Trajectory TrajectoryUtil::DeserializeTrajectory(std::string_view jsonStr) {
  wpi::json json = wpi::json::parse(jsonStr);
  return Trajectory{json.get<std::vector<Trajectory::State>>()};
}
