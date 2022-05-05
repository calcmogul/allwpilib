// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <limits>

#include <wpi/ConstexprMath.h>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>

#include "frc/geometry/CoordinateAxis.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"

namespace frc {

/**
 * A helper class that converts Pose3d objects between different standard
 * coordinate frames.
 */
class WPILIB_DLLEXPORT CoordinateSystem {
 public:
  static constexpr CoordinateAxis N{1.0, 0.0, 0.0};   // +X in NWU
  static constexpr CoordinateAxis S{-1.0, 0.0, 0.0};  // -X in NWU
  static constexpr CoordinateAxis E{0.0, -1.0, 0.0};  // -Y in NWU
  static constexpr CoordinateAxis W{0.0, 1.0, 0.0};   // +Y in NWU
  static constexpr CoordinateAxis U{0.0, 0.0, 1.0};   // +Z in NWU
  static constexpr CoordinateAxis D{0.0, 0.0, -1.0};  // -Z in NWU

  // Common coordinate systems
  static constexpr CoordinateSystem NWU() { return CoordinateSystem{N, W, U}; }
  static constexpr CoordinateSystem EDN() { return CoordinateSystem{E, D, N}; }
  static constexpr CoordinateSystem NED() { return CoordinateSystem{N, E, D}; }

  /**
   * Constructs a coordinate system with the given cardinal directions for each
   * axis.
   *
   * @param positiveX The cardinal direction of the positive x-axis.
   * @param positiveY The cardinal direction of the positive y-axis.
   * @param positiveZ The cardinal direction of the positive z-axis.
   * @throws std::domain_error if the coordinate system isn't special orthogonal
   */
  constexpr CoordinateSystem(CoordinateAxis positiveX, CoordinateAxis positiveY,
                             CoordinateAxis positiveZ) {
    // Construct a change of basis matrix from the source coordinate system to
    // the NWU coordinate system. Each column vector in the change of basis
    // matrix is one of the old basis vectors mapped to its representation in
    // the new basis.
    Matrix<3, 3> R{
        {positiveX.m_axis[0], positiveY.m_axis[0], positiveZ.m_axis[0]},
        {positiveX.m_axis[1], positiveY.m_axis[1], positiveZ.m_axis[1]},
        {positiveX.m_axis[2], positiveY.m_axis[2], positiveZ.m_axis[2]}};

    // Require that the change of basis matrix is special orthogonal. This is
    // true if the axes used are orthogonal and normalized. The Axis class
    // already normalizes itself, so we just need to check for orthogonality.
    if (R * R.Transpose() != Matrix<3, 3>::Identity()) {
      throw std::domain_error("Coordinate system isn't special orthogonal");
    }

    // Turn change of basis matrix into a quaternion since it's a pure rotation
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      w = 0.25 / s;
      x = (R(2, 1) - R(1, 2)) * s;
      y = (R(0, 2) - R(2, 0)) * s;
      z = (R(1, 0) - R(0, 1)) * s;
    } else {
      if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
        w = (R(2, 1) - R(1, 2)) / s;
        x = 0.25 * s;
        y = (R(0, 1) + R(1, 0)) / s;
        z = (R(0, 2) + R(2, 0)) / s;
      } else if (R(1, 1) > R(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
        w = (R(0, 2) - R(2, 0)) / s;
        x = (R(0, 1) + R(1, 0)) / s;
        y = 0.25 * s;
        z = (R(1, 2) + R(2, 1)) / s;
      } else {
        double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
        w = (R(1, 0) - R(0, 1)) / s;
        x = (R(0, 2) + R(2, 0)) / s;
        y = (R(1, 2) + R(2, 1)) / s;
        z = 0.25 * s;
      }
    }

    m_rotation = Rotation3d{Quaternion{w, x, y, z}};
  }

  constexpr CoordinateSystem(const CoordinateSystem&) = default;
  constexpr CoordinateSystem& operator=(const CoordinateSystem&) = default;

  constexpr CoordinateSystem(CoordinateSystem&&) = default;
  constexpr CoordinateSystem& operator=(CoordinateSystem&&) = default;

  /**
   * Converts the given pose from one coordinate system to another.
   *
   * @param pose The pose to convert.
   * @param from The coordinate system the pose starts in.
   * @param to The coordinate system to which to convert.
   * @return The given pose in the desired coordinate system.
   */
  static Pose3d Convert(const Pose3d& pose, const CoordinateSystem& from,
                        const CoordinateSystem& to);

 private:
  // Rotation from this coordinate system to NWU coordinate system
  Rotation3d m_rotation;
};

}  // namespace frc
