// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <limits>

namespace wpi {
namespace detail {

constexpr double sqrtNewtonRaphson(double x, double curr, double prev) {
  if (curr == prev) {
    return curr;
  } else {
    return sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
  }
}

}  // namespace detail

constexpr double abs(double x) {
  return x > 0.0 ? x : -x;
}

constexpr double sqrt(double x) {
  if (x >= 0.0 && x < std::numeric_limits<double>::infinity()) {
    return detail::sqrtNewtonRaphson(x, x, 0.0);
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

constexpr double hypot(double x, double y) {
  return wpi::sqrt(x * x + y * y);
}

constexpr double hypot(double x, double y, double z) {
  return wpi::sqrt(x * x + y * y + z * z);
}

}  // namespace wpi
