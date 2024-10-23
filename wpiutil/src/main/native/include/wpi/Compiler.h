// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#ifndef WPI_ALWAYS_INLINE
#if __has_attribute(always_inline)
#define WPI_ALWAYS_INLINE inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define WPI_ALWAYS_INLINE __forceinline
#else
#define WPI_ALWAYS_INLINE inline
#endif
#endif
