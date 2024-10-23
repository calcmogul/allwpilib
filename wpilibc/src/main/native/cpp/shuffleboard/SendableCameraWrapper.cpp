// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/shuffleboard/SendableCameraWrapper.h"

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

namespace frc {
namespace detail {
std::shared_ptr<SendableCameraWrapper>& GetSendableCameraWrapper(
    std::string_view cameraName) {
  static std::map<std::string, std::shared_ptr<SendableCameraWrapper>,
                  std::less<>>
      wrappers;
  return wrappers[cameraName];
}

void AddToSendableRegistry(wpi::Sendable* sendable, std::string_view name) {
  wpi::SendableRegistry::Add(sendable, name);
}
}  // namespace detail

void SendableCameraWrapper::InitSendable(wpi::SendableBuilder& builder) {
  builder.AddStringProperty(
      ".ShuffleboardURI", [this] { return m_uri; }, nullptr);
}
}  // namespace frc
