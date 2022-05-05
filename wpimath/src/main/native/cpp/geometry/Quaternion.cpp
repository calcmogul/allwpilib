// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Quaternion.h"

using namespace frc;

Eigen::Vector3d Quaternion::ToRotationVector() const {
  // See equation (31) in "Integrating Generic Sensor Fusion Algorithms with
  // Sound State Representation through Encapsulation of Manifolds"
  //
  // https://arxiv.org/pdf/1107.1119.pdf
  double norm = m_v.Norm();

  if (norm < 1e-9) {
    return Eigen::Vector3d{
        (2.0 / W() - 2.0 / 3.0 * norm * norm / (W() * W() * W())) * m_v};
  } else {
    if (W() < 0.0) {
      return Eigen::Vector3d{2.0 * std::atan2(-norm, -W()) / norm * m_v};
    } else {
      return Eigen::Vector3d{2.0 * std::atan2(norm, W()) / norm * m_v};
    }
  }
}
