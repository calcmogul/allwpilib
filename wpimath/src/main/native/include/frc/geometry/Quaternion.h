// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <glaze/json.hpp>
#include <wpi/SymbolExports.h>

namespace frc {

class WPILIB_DLLEXPORT Quaternion {
 public:
  /**
   * Constructs a quaternion with a default angle of 0 degrees.
   */
  Quaternion() = default;

  /**
   * Constructs a quaternion with the given components.
   *
   * @param w W component of the quaternion.
   * @param x X component of the quaternion.
   * @param y Y component of the quaternion.
   * @param z Z component of the quaternion.
   */
  Quaternion(double w, double x, double y, double z);

  /**
   * Multiply with another quaternion.
   *
   * @param other The other quaternion.
   */
  Quaternion operator*(const Quaternion& other) const;

  /**
   * Checks equality between this Quaternion and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Quaternion& other) const;

  /**
   * Returns the inverse of the quaternion.
   */
  Quaternion Inverse() const;

  /**
   * Normalizes the quaternion.
   */
  Quaternion Normalize() const;

  /**
   * Returns W component of the quaternion.
   */
  double W() const;

  /**
   * Returns X component of the quaternion.
   */
  double X() const;

  /**
   * Returns Y component of the quaternion.
   */
  double Y() const;

  /**
   * Returns Z component of the quaternion.
   */
  double Z() const;

  /**
   * Returns the rotation vector representation of this quaternion.
   *
   * This is also the log operator of SO(3).
   */
  Eigen::Vector3d ToRotationVector() const;

 private:
  // Scalar r in versor form
  double m_r = 1.0;

  // Vector v in versor form
  Eigen::Vector3d m_v{0.0, 0.0, 0.0};
};

}  // namespace frc

namespace glz::detail {

template <>
struct from_json<frc::Quaternion> {
  template <auto Opts>
  static void op(frc::Quaternion& value, auto&&... args) {
    double w;
    double x;
    double y;
    double z;

    read<json>::op<Opts>(w, args...);
    read<json>::op<Opts>(x, args...);
    read<json>::op<Opts>(y, args...);
    read<json>::op<Opts>(z, args...);

    value = frc::Quaternion{w, x, y, z};
  }
};

template <>
struct to_json<frc::Quaternion> {
  template <auto Opts>
  static void op(const frc::Quaternion& value, auto&&... args) noexcept {
    write<json>::op<Opts>(value.W(), args...);
    write<json>::op<Opts>(value.X(), args...);
    write<json>::op<Opts>(value.Y(), args...);
    write<json>::op<Opts>(value.Z(), args...);
  }
};

}  // namespace glz::detail
