// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <wpi/UidVector.h>
#include <wpi/flat_map.h>

#include "wpinet/MulticastServiceAnnouncer.h"
#include "wpinet/MulticastServiceResolver.h"

namespace wpi {
struct MulticastHandleManager {
  wpi::mutex mutex;
  wpi::UidVector<int, 8> handleIds;
  wpi::flat_map<size_t, std::unique_ptr<wpi::MulticastServiceResolver>>
      resolvers;
  wpi::flat_map<size_t, std::unique_ptr<wpi::MulticastServiceAnnouncer>>
      announcers;
#ifdef _WIN32
  ~MulticastHandleManager();
#endif
};

MulticastHandleManager& GetMulticastManager();
}  // namespace wpi
