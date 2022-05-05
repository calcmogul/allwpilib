// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/ConstexprMath.h>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>

namespace frc {

/**
 * A class representing a coordinate system axis within the NWU coordinate
 * system.
 */
class WPILIB_DLLEXPORT CoordinateAxis {
 public:
  /**
   * Constructs a coordinate system axis within the NWU coordinate system and
   * normalizes it.
   *
   * @param x The x component.
   * @param y The y component.
   * @param z The z component.
   */
  constexpr CoordinateAxis(double x, double y, double z) : m_axis{x, y, z} {
    double norm = wpi::sqrt(x * x + y * y + z * z);
    m_axis[0] /= norm;
    m_axis[1] /= norm;
    m_axis[2] /= norm;
  }

  CoordinateAxis(const CoordinateAxis&) = default;
  CoordinateAxis& operator=(const CoordinateAxis&) = default;

  CoordinateAxis(CoordinateAxis&&) = default;
  CoordinateAxis& operator=(CoordinateAxis&&) = default;

 private:
  friend class CoordinateSystem;

  wpi::array<double, 3> m_axis;
};

}  // namespace frc
