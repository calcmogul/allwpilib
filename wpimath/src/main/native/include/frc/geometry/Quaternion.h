// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/ConstexprMath.h>
#include <wpi/SymbolExports.h>

#include "frc/EigenCore.h"
#include "frc/Matrix.h"

namespace frc {

class WPILIB_DLLEXPORT Quaternion {
 public:
  /**
   * Constructs a quaternion with a default angle of 0 degrees.
   */
  constexpr Quaternion() = default;

  /**
   * Constructs a quaternion with the given components.
   *
   * @param w W component of the quaternion.
   * @param x X component of the quaternion.
   * @param y Y component of the quaternion.
   * @param z Z component of the quaternion.
   */
  constexpr Quaternion(double w, double x, double y, double z)
      : m_r{w}, m_v{x, y, z} {}

  /**
   * Multiply with another quaternion.
   *
   * @param other The other quaternion.
   */
  constexpr Quaternion operator*(const Quaternion& other) const {
    // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    const auto& r1 = m_r;
    const auto& v1 = m_v;
    const auto& r2 = other.m_r;
    const auto& v2 = other.m_v;

    Vector<3> cross{v1(1) * v2(2) - v2(1) * v1(2),
                    v2(0) * v1(2) - v1(0) * v2(2),
                    v1(0) * v2(1) - v2(0) * v1(1)};

    Vector<3> v = r1 * v2 + r2 * v1 + cross;
    return Quaternion{r1 * r2 - (v1.Transpose() * v2)(0), v(0), v(1), v(2)};
  }

  /**
   * Checks equality between this Quaternion and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Quaternion& other) const {
    return wpi::abs(m_r * other.m_r + (m_v.Transpose() * other.m_v)(0)) >
           1.0 - 1E-9;
  }

  /**
   * Checks inequality between this Quaternion and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  constexpr bool operator!=(const Quaternion& other) const {
    return !operator==(other);
  }

  /**
   * Returns the inverse of the quaternion.
   */
  constexpr Quaternion Inverse() const {
    return Quaternion{m_r, -m_v(0), -m_v(1), -m_v(2)};
  }

  /**
   * Normalizes the quaternion.
   */
  constexpr Quaternion Normalize() const {
    double norm = wpi::sqrt(W() * W() + X() * X() + Y() * Y() + Z() * Z());
    if (norm == 0.0) {
      return Quaternion{};
    } else {
      return Quaternion{W() / norm, X() / norm, Y() / norm, Z() / norm};
    }
  }

  /**
   * Returns W component of the quaternion.
   */
  constexpr double W() const { return m_r; }

  /**
   * Returns X component of the quaternion.
   */
  constexpr double X() const { return m_v(0); }

  /**
   * Returns Y component of the quaternion.
   */
  constexpr double Y() const { return m_v(1); }

  /**
   * Returns Z component of the quaternion.
   */
  constexpr double Z() const { return m_v(2); }

  /**
   * Returns the rotation vector representation of this quaternion.
   *
   * This is also the log operator of SO(3).
   */
  Eigen::Vector3d ToRotationVector() const;

 private:
  double m_r = 1.0;
  Vector<3> m_v{0.0, 0.0, 0.0};
};

}  // namespace frc
