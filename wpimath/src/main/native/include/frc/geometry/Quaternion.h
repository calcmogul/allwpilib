// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <Eigen/Core>
#include <gcem.hpp>
#include <wpi/SymbolExports.h>
#include <wpi/json_fwd.h>

namespace frc {

/**
 * Represents a quaternion.
 */
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
   * Adds with another quaternion.
   *
   * @param other the other quaternion
   */
  constexpr Quaternion operator+(const Quaternion& other) const {
    return Quaternion{
        m_r + other.m_r,
        m_v(0) + other.m_v(0),
        m_v(1) + other.m_v(1),
        m_v(2) + other.m_v(2),
    };
  }

  /**
   * Subtracts another quaternion.
   *
   * @param other the other quaternion
   */
  constexpr Quaternion operator-(const Quaternion& other) const {
    return Quaternion{
        m_r - other.m_r,
        m_v(0) - other.m_v(0),
        m_v(1) - other.m_v(1),
        m_v(2) - other.m_v(2),
    };
  }

  /**
   * Multiples with a scalar value.
   *
   * @param other the scalar value
   */
  constexpr Quaternion operator*(double other) const {
    return Quaternion{
        m_r * other,
        m_v(0) * other,
        m_v(1) * other,
        m_v(2) * other,
    };
  }

  /**
   * Divides by a scalar value.
   *
   * @param other the scalar value
   */
  constexpr Quaternion operator/(double other) const {
    return Quaternion{
        m_r / other,
        m_v(0) / other,
        m_v(1) / other,
        m_v(2) / other,
    };
  }

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

    // v₁ x v₂
    Eigen::Vector3d cross{v1(1) * v2(2) - v2(1) * v1(2),
                          v2(0) * v1(2) - v1(0) * v2(2),
                          v1(0) * v2(1) - v2(0) * v1(1)};

    // v = r₁v₂ + r₂v₁ + v₁ x v₂
    Eigen::Vector3d v = r1 * v2 + r2 * v1 + cross;

    return Quaternion{// r = r₁r₂ − v₁ ⋅ v₂
                      r1 * r2 - v1.dot(v2),
                      // v = r₁v₂ + r₂v₁ + v₁ x v₂
                      v(0), v(1), v(2)};
  }

  /**
   * Checks equality between this Quaternion and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Quaternion& other) const {
    return std::abs(Dot(other) - Norm() * other.Norm()) < 1e-9 &&
           std::abs(Norm() - other.Norm()) < 1e-9;
  }

  /**
   * Returns the elementwise product of two quaternions.
   */
  constexpr double Dot(const Quaternion& other) const {
    return W() * other.W() + m_v.dot(other.m_v);
  }

  /**
   * Returns the conjugate of the quaternion.
   */
  constexpr Quaternion Conjugate() const {
    return Quaternion{W(), -X(), -Y(), -Z()};
  }

  /**
   * Returns the inverse of the quaternion.
   */
  constexpr Quaternion Inverse() const {
    double norm = Norm();
    return Conjugate() / (norm * norm);
  }

  /**
   * Normalizes the quaternion.
   */
  constexpr Quaternion Normalize() const {
    double norm = Norm();
    if (norm == 0.0) {
      return Quaternion{};
    } else {
      return Quaternion{W(), X(), Y(), Z()} / norm;
    }
  }

  /**
   * Calculates the L2 norm of the quaternion.
   */
  constexpr double Norm() const { return std::sqrt(Dot(*this)); }

  /**
   * Calculates this quaternion raised to a power.
   *
   * @param t the power to raise this quaternion to.
   */
  constexpr Quaternion Pow(double t) const { return (Log() * t).Exp(); }

  /**
   * Matrix exponential of a quaternion.
   *
   * @param other the "Twist" that will be applied to this quaternion.
   */
  constexpr Quaternion Exp(const Quaternion& other) const {
    return other.Exp() * *this;
  }

  /**
   * Matrix exponential of a quaternion.
   *
   * source: wpimath/algorithms.md
   *
   *  If this quaternion is in 𝖘𝖔(3) and you are looking for an element of
   * SO(3), use FromRotationVector
   */
  constexpr Quaternion Exp() const {
    double scalar = gcem::exp(m_r);

    double axial_magnitude = m_v.norm();
    double cosine = gcem::cos(axial_magnitude);

    double axial_scalar;

    if (axial_magnitude < 1e-9) {
      // Taylor series of sin(x)/x near x=0: 1 − x²/6 + x⁴/120 + O(n⁶)
      double axial_magnitude_sq = axial_magnitude * axial_magnitude;
      double axial_magnitude_sq_sq = axial_magnitude_sq * axial_magnitude_sq;
      axial_scalar =
          1.0 - axial_magnitude_sq / 6.0 + axial_magnitude_sq_sq / 120.0;
    } else {
      axial_scalar = gcem::sin(axial_magnitude) / axial_magnitude;
    }

    return Quaternion(cosine * scalar, X() * axial_scalar * scalar,
                      Y() * axial_scalar * scalar, Z() * axial_scalar * scalar);
  }

  /**
   * Log operator of a quaternion.
   *
   * @param other The quaternion to map this quaternion onto
   */
  constexpr Quaternion Log(const Quaternion& other) const {
    return (other * Inverse()).Log();
  }

  /**
   * Log operator of a quaternion.
   *
   * source:  wpimath/algorithms.md
   *
   * If this quaternion is in SO(3) and you are looking for an element of 𝖘𝖔(3),
   * use ToRotationVector
   */
  constexpr Quaternion Log() const {
    double scalar = std::log(Norm());

    double v_norm = m_v.norm();

    double s_norm = W() / Norm();

    if (std::abs(s_norm + 1) < 1e-9) {
      return Quaternion{scalar, -std::numbers::pi, 0, 0};
    }

    double v_scalar;

    if (v_norm < 1e-9) {
      // Taylor series expansion of atan2(y / x) / y around y = 0 = 1/x -
      // y^2/3*x^3 + O(y^4)
      v_scalar = 1.0 / W() - 1.0 / 3.0 * v_norm * v_norm / (W() * W() * W());
    } else {
      v_scalar = std::atan2(v_norm, W()) / v_norm;
    }

    return Quaternion{scalar, v_scalar * m_v(0), v_scalar * m_v(1),
                      v_scalar * m_v(2)};
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
  constexpr Eigen::Vector3d ToRotationVector() const {
    // See equation (31) in "Integrating Generic Sensor Fusion Algorithms with
    // Sound State Representation through Encapsulation of Manifolds"
    //
    // https://arxiv.org/pdf/1107.1119.pdf
    double norm = m_v.norm();

    if (norm < 1e-9) {
      return (2.0 / W() - 2.0 / 3.0 * norm * norm / (W() * W() * W())) * m_v;
    } else {
      if (W() < 0.0) {
        return 2.0 * gcem::atan2(-norm, -W()) / norm * m_v;
      } else {
        return 2.0 * gcem::atan2(norm, W()) / norm * m_v;
      }
    }
  }

  /**
   * Returns the quaternion representation of this rotation vector.
   *
   * This is also the exp operator of 𝖘𝖔(3).
   *
   * source: wpimath/algorithms.md
   */
  constexpr static Quaternion FromRotationVector(const Eigen::Vector3d& rvec) {
    // 𝑣⃗ = θ * v̂
    // v̂ = 𝑣⃗ / θ

    // 𝑞 = std::cos(θ/2) + std::sin(θ/2) * v̂
    // 𝑞 = std::cos(θ/2) + std::sin(θ/2) / θ * 𝑣⃗

    double theta = rvec.norm();
    double cos = gcem::cos(theta / 2);

    double axial_scalar;

    if (theta < 1e-9) {
      // taylor series expansion of sin(θ/2) / θ around θ = 0 = 1/2 - θ²/48 +
      // O(θ⁴)
      axial_scalar = 1.0 / 2.0 - theta * theta / 48.0;
    } else {
      axial_scalar = gcem::sin(theta / 2) / theta;
    }

    return Quaternion{cos, axial_scalar * rvec(0), axial_scalar * rvec(1),
                      axial_scalar * rvec(2)};
  }

 private:
  // Scalar r in versor form
  double m_r = 1.0;

  // Vector v in versor form
  Eigen::Vector3d m_v{0.0, 0.0, 0.0};
};

WPILIB_DLLEXPORT
void to_json(wpi::json& json, const Quaternion& quaternion);

WPILIB_DLLEXPORT
void from_json(const wpi::json& json, Quaternion& quaternion);

}  // namespace frc

#include "frc/geometry/proto/QuaternionProto.h"
#include "frc/geometry/struct/QuaternionStruct.h"
