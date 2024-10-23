// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//===- llvm/Support/Errno.h - Portable+convenient errno handling -*- C++
//-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares some portable and convenient functions to deal with errno.
//
//===----------------------------------------------------------------------===//

#ifndef WPIUTIL_WPI_ERRNO_H_
#define WPIUTIL_WPI_ERRNO_H_

#include <cerrno>

namespace wpi::sys {

template <typename FailT, typename Fun, typename... Args>
inline decltype(auto) RetryAfterSignal(const FailT& Fail, const Fun& F,
                                       const Args&... As) {
  decltype(F(As...)) Res;
  do {
    errno = 0;
    Res = F(As...);
  } while (Res == Fail && errno == EINTR);
  return Res;
}

}  // namespace wpi::sys

#endif  // WPIUTIL_WPI_ERRNO_H_
