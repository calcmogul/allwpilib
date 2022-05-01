// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/EigenCore.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"

namespace frc {

/**
 * A helper class that converts Pose3d objects between different standard
 * coordinate frames.
 */
class WPILIB_DLLEXPORT CoordinateSystem {
 public:
  /**
   * A class representing a coordinate system axis within the NWU coordinate
   * system.
   */
  class WPILIB_DLLEXPORT Axis {
   public:
    /**
     * Constructs a coordinate system axis within the NWU coordinate system and
     * normalizes it.
     *
     * @param x The x component.
     * @param y The y component.
     * @param z The z component.
     */
    Axis(double x, double y, double z);

    Axis(const Axis&) = default;
    Axis& operator=(const Axis&) = default;
    Axis(Axis&&) = default;
    Axis& operator=(Axis&&) = default;

   private:
    friend class CoordinateSystem;

    Vectord<3> m_axis;
  };

  static const Axis N;
  static const Axis S;
  static const Axis E;
  static const Axis W;
  static const Axis U;
  static const Axis D;

  // Common coordinate systems
  static const CoordinateSystem NWU;
  static const CoordinateSystem EDN;
  static const CoordinateSystem NED;

  /**
   * Constructs a coordinate system with the given cardinal directions for each
   * axis.
   *
   * @param positiveX The cardinal direction of the positive x-axis.
   * @param positiveY The cardinal direction of the positive y-axis.
   * @param positiveZ The cardinal direction of the positive z-axis.
   * @throws std::domain_error if the coordinate system isn't special orthogonal
   */
  CoordinateSystem(Axis positiveX, Axis positiveY, Axis positiveZ);

  CoordinateSystem(const CoordinateSystem&) = default;
  CoordinateSystem& operator=(const CoordinateSystem&) = default;
  CoordinateSystem(CoordinateSystem&&) = default;
  CoordinateSystem& operator=(CoordinateSystem&&) = default;

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
