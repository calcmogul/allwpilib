// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/CoordinateSystem.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "Eigen/QR"

using namespace frc;

Pose3d CoordinateSystem::Convert(const Pose3d& pose,
                                 const CoordinateSystem& from,
                                 const CoordinateSystem& to) {
  return pose.RelativeTo(
      Pose3d{Translation3d{}, to.m_rotation - from.m_rotation});
}
