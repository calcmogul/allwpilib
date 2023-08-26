// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/TrajectoryUtil.h"

#include <system_error>

#include <fmt/format.h>
#include <glaze/json.hpp>
#include <wpi/MemoryBuffer.h>
#include <wpi/SmallString.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

using namespace frc;

void TrajectoryUtil::ToPathweaverJson(const Trajectory& trajectory,
                                      std::string_view path) {
  std::error_code error_code;

  wpi::raw_fd_ostream output{path, error_code};
  if (error_code) {
    throw std::runtime_error(fmt::format("Cannot open file: {}", path));
  }

  std::string json = glz::write_json(trajectory.States());
  output << json;
  output.flush();
}

Trajectory TrajectoryUtil::FromPathweaverJson(std::string_view path) {
  std::error_code ec;
  std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
      wpi::MemoryBuffer::GetFile(path, ec);

  if (fileBuffer == nullptr || ec) {
    throw std::runtime_error(fmt::format("Cannot open file: {}", path));
  }

  std::string_view fileContents{
      reinterpret_cast<const char*>(fileBuffer->begin()), fileBuffer->size()};

  return Trajectory{
      glz::read_json<std::vector<Trajectory::State>>(fileContents).value()};
}

std::string TrajectoryUtil::SerializeTrajectory(const Trajectory& trajectory) {
  return glz::write_json(trajectory.States());
}

Trajectory TrajectoryUtil::DeserializeTrajectory(std::string_view jsonStr) {
  return Trajectory{
      glz::read_json<std::vector<Trajectory::State>>(jsonStr).value()};
}
