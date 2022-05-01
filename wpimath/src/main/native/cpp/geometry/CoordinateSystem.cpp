// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/CoordinateSystem.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "Eigen/QR"

using namespace frc;

namespace {

CoordinateSystem::Axis MakeN() {
  static CoordinateSystem::Axis axis{1.0, 0.0, 0.0};  // +X in NWU
  return axis;
}

CoordinateSystem::Axis MakeS() {
  static CoordinateSystem::Axis axis{-1.0, 0.0, 0.0};  // -X in NWU
  return axis;
}

CoordinateSystem::Axis MakeE() {
  static CoordinateSystem::Axis axis{0.0, -1.0, 0.0};  // -Y in NWU
  return axis;
}

CoordinateSystem::Axis MakeW() {
  static CoordinateSystem::Axis axis{0.0, 1.0, 0.0};  // +Y in NWU
  return axis;
}

CoordinateSystem::Axis MakeU() {
  static CoordinateSystem::Axis axis{0.0, 0.0, 1.0};  // +Z in NWU
  return axis;
}

CoordinateSystem::Axis MakeD() {
  static CoordinateSystem::Axis axis{0.0, 0.0, -1.0};  // -Z in NWU
  return axis;
}

}  // namespace

// Map each cardinal direction to an axis in the NWU coordinate system
const CoordinateSystem::Axis CoordinateSystem::N = MakeN();
const CoordinateSystem::Axis CoordinateSystem::S = MakeS();
const CoordinateSystem::Axis CoordinateSystem::E = MakeE();
const CoordinateSystem::Axis CoordinateSystem::W = MakeW();
const CoordinateSystem::Axis CoordinateSystem::U = MakeU();
const CoordinateSystem::Axis CoordinateSystem::D = MakeD();

// Factory functions are used instead of the static class instances to avoid
// static init order races
const CoordinateSystem CoordinateSystem::NWU{MakeN(), MakeW(), MakeU()};
const CoordinateSystem CoordinateSystem::EDN{MakeE(), MakeD(), MakeN()};
const CoordinateSystem CoordinateSystem::NED{MakeN(), MakeE(), MakeD()};

CoordinateSystem::Axis::Axis(double x, double y, double z) : m_axis{x, y, z} {
  m_axis /= m_axis.norm();
}

CoordinateSystem::CoordinateSystem(Axis positiveX, Axis positiveY,
                                   Axis positiveZ) {
  // Construct a change of basis matrix from the source coordinate system to the
  // NWU coordinate system. Each column vector in the change of basis matrix is
  // one of the old basis vectors mapped to its representation in the new basis.
  Matrixd<3, 3> R;
  R.block<3, 1>(0, 0) = positiveX.m_axis;
  R.block<3, 1>(0, 1) = positiveY.m_axis;
  R.block<3, 1>(0, 2) = positiveZ.m_axis;

  // Require that the change of basis matrix is special orthogonal. This is true
  // if the axes used are orthogonal and normalized. The Axis class already
  // normalizes itself, so we just need to check for orthogonality.
  if (R * R.transpose() != Matrixd<3, 3>::Identity()) {
    throw std::domain_error("Coordinate system isn't special orthogonal");
  }

  // Turn change of basis matrix into a quaternion since it's a pure rotation
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
  double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double w;
  double x;
  double y;
  double z;

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

Pose3d CoordinateSystem::Convert(const Pose3d& pose,
                                 const CoordinateSystem& from,
                                 const CoordinateSystem& to) {
  return pose.RelativeTo(
      Pose3d{Translation3d{}, to.m_rotation - from.m_rotation});
}
