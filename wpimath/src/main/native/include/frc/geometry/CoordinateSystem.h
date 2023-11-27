// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/geometry/CoordinateAxis.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"

namespace frc {

/**
 * A helper class that converts Pose3d objects between different standard
 * coordinate frames.
 */
class WPILIB_DLLEXPORT CoordinateSystem {
 public:
  /**
   * Constructs a coordinate system with the given cardinal directions for each
   * axis.
   *
   * @param positiveX The cardinal direction of the positive x-axis.
   * @param positiveY The cardinal direction of the positive y-axis.
   * @param positiveZ The cardinal direction of the positive z-axis.
   * @throws std::domain_error if the coordinate system isn't special orthogonal
   */
  constexpr CoordinateSystem(const CoordinateAxis& positiveX,
                             const CoordinateAxis& positiveY,
                             const CoordinateAxis& positiveZ) {
    // Construct a change of basis matrix from the source coordinate system to
    // the NWU coordinate system. Each column vector in the change of basis
    // matrix is one of the old basis vectors mapped to its representation in
    // the new basis.
    Eigen::Matrix3d R;
    R.block<3, 1>(0, 0) = positiveX.m_axis;
    R.block<3, 1>(0, 1) = positiveY.m_axis;
    R.block<3, 1>(0, 2) = positiveZ.m_axis;

    // The change of basis matrix should be a pure rotation. The Rotation3d
    // constructor will verify this by checking for special orthogonality.
    m_rotation = Rotation3d{R};
  }

  constexpr CoordinateSystem(const CoordinateSystem&) = default;
  constexpr CoordinateSystem& operator=(const CoordinateSystem&) = default;
  constexpr CoordinateSystem(CoordinateSystem&&) = default;
  constexpr CoordinateSystem& operator=(CoordinateSystem&&) = default;

  /**
   * Returns an instance of the North-West-Up (NWU) coordinate system.
   *
   * The +X axis is north, the +Y axis is west, and the +Z axis is up.
   */
  static constexpr CoordinateSystem NWU() {
    return CoordinateSystem{CoordinateAxis::N(), CoordinateAxis::W(),
                            CoordinateAxis::U()};
  }

  /**
   * Returns an instance of the East-Down-North (EDN) coordinate system.
   *
   * The +X axis is east, the +Y axis is down, and the +Z axis is north.
   */
  static constexpr CoordinateSystem EDN() {
    return CoordinateSystem{CoordinateAxis::E(), CoordinateAxis::D(),
                            CoordinateAxis::N()};
  }

  /**
   * Returns an instance of the NED coordinate system.
   *
   * The +X axis is north, the +Y axis is east, and the +Z axis is down.
   */
  static constexpr CoordinateSystem NED() {
    return CoordinateSystem{CoordinateAxis::N(), CoordinateAxis::E(),
                            CoordinateAxis::D()};
  }

  /**
   * Converts the given translation from one coordinate system to another.
   *
   * @param translation The translation to convert.
   * @param from The coordinate system the translation starts in.
   * @param to The coordinate system to which to convert.
   * @return The given translation in the desired coordinate system.
   */
  static constexpr Translation3d Convert(const Translation3d& translation,
                                         const CoordinateSystem& from,
                                         const CoordinateSystem& to) {
    return translation.RotateBy(from.m_rotation - to.m_rotation);
  }

  /**
   * Converts the given rotation from one coordinate system to another.
   *
   * @param rotation The rotation to convert.
   * @param from The coordinate system the rotation starts in.
   * @param to The coordinate system to which to convert.
   * @return The given rotation in the desired coordinate system.
   */
  static constexpr Rotation3d Convert(const Rotation3d& rotation,
                                      const CoordinateSystem& from,
                                      const CoordinateSystem& to) {
    return rotation.RotateBy(from.m_rotation - to.m_rotation);
  }

  /**
   * Converts the given pose from one coordinate system to another.
   *
   * @param pose The pose to convert.
   * @param from The coordinate system the pose starts in.
   * @param to The coordinate system to which to convert.
   * @return The given pose in the desired coordinate system.
   */
  static constexpr Pose3d Convert(const Pose3d& pose,
                                  const CoordinateSystem& from,
                                  const CoordinateSystem& to) {
    return Pose3d{Convert(pose.Translation(), from, to),
                  Convert(pose.Rotation(), from, to)};
  }

  /**
   * Converts the given transform from one coordinate system to another.
   *
   * @param transform The transform to convert.
   * @param from The coordinate system the transform starts in.
   * @param to The coordinate system to which to convert.
   * @return The given transform in the desired coordinate system.
   */
  static constexpr Transform3d Convert(const Transform3d& transform,
                                       const CoordinateSystem& from,
                                       const CoordinateSystem& to) {
    const auto coordRot = from.m_rotation - to.m_rotation;
    return Transform3d{
        Convert(transform.Translation(), from, to),
        (-coordRot).RotateBy(transform.Rotation().RotateBy(coordRot))};
  }

 private:
  // Rotation from this coordinate system to NWU coordinate system
  Rotation3d m_rotation;
};

}  // namespace frc
