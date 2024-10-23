// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <cassert>

namespace wpi {

/// Returns the next integer (mod 2**64) that is greater than or equal to
/// \p Value and is a multiple of \p Align. \p Align must be non-zero.
///
/// Examples:
/// \code
///   alignTo(5, 8) = 8
///   alignTo(17, 8) = 24
///   alignTo(~0LL, 8) = 0
///   alignTo(321, 255) = 510
/// \endcode
inline uint64_t alignTo(uint64_t Value, uint64_t Align) {
  assert(Align != 0u && "Align can't be 0.");
  return (Value + Align - 1) / Align * Align;
}

// Typesafe implementation of the signum function.
// Returns -1 if negative, 1 if positive, 0 if 0.
template <typename T>
constexpr int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * Linearly interpolates between two values.
 *
 * @param startValue The start value.
 * @param endValue The end value.
 * @param t The fraction for interpolation.
 *
 * @return The interpolated value.
 */
template <typename T>
constexpr T Lerp(const T& startValue, const T& endValue, double t) {
  return startValue + (endValue - startValue) * t;
}

}  // namespace wpi
