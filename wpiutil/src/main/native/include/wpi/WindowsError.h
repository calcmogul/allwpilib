// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//===-- WindowsError.h - Support for mapping windows errors to posix-------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef WPIUTIL_WPI_WINDOWSERROR_H_
#define WPIUTIL_WPI_WINDOWSERROR_H_

#include <system_error>

namespace wpi {
std::error_code mapWindowsError(unsigned EV);
}  // namespace wpi

#endif  // WPIUTIL_WPI_WINDOWSERROR_H_
