// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <wpi/small_vector.h>

#include "glass/Context.h"
#include "glass/Storage.h"

namespace glass {

class DataSource;

class Context {
 public:
  Context();
  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;
  ~Context();

  std::vector<std::function<void()>> workspaceInit;
  std::vector<std::function<void()>> workspaceReset;
  std::string storageLoadDir = ".";
  std::string storageAutoSaveDir = ".";
  std::string storageName = "imgui";
  wpi::small_vector<Storage*, 32> storageStack;
  std::map<std::string, std::unique_ptr<Storage>, std::less<>> storageRoots;
  std::map<std::string, bool, std::less<>> deviceHidden;
  std::map<std::string, DataSource*, std::less<>> sources;
  Storage& sourceNameStorage;
  uint64_t zeroTime = 0;
  bool isPlatformSaveDir = false;
};

extern Context* gContext;

}  // namespace glass
