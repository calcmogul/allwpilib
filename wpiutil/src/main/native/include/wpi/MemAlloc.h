// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/print.h>

#include <cstdlib>

namespace wpi {

inline void* safe_malloc(size_t size) {
  void* result = std::malloc(size);

  if (result == nullptr) {
    if (size == 0) {
      // It is implementation-defined whether allocation occurs if the space
      // requested is zero (ISO/IEC 9899:2018 7.22.3). Retry, requesting
      // non-zero, if the space requested was zero.
      return safe_malloc(1);
    }
    wpi::println(stderr, "Allocation failed");
    std::abort();
  }

  return result;
}

}  // namespace wpi
