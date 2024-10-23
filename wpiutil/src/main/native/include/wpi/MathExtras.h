// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace wpi {

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
