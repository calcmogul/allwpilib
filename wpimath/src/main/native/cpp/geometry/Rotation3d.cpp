// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Rotation3d.h"

#include <Eigen/QR>
#include <wpi/json.h>

using namespace frc;

Rotation3d::Rotation3d(const Eigen::Vector3d& initial,
                       const Eigen::Vector3d& final) {
  double dot =
      initial(0) * final(0) + initial(1) * final(1) + initial(2) * final(2);
  double normProduct = gcem::hypot(initial(0), initial(1), initial(2)) *
                       gcem::hypot(final(0), final(1), final(2));
  double dotNorm = dot / normProduct;

  if (dotNorm > 1.0 - 1E-9) {
    // If the dot product is 1, the two vectors point in the same direction so
    // there's no rotation. The default initialization of m_q will work.
    return;
  } else if (dotNorm < -1.0 + 1E-9) {
    // If the dot product is -1, the two vectors point in opposite directions so
    // a 180 degree rotation is required. Any orthogonal vector can be used for
    // it. Q in the QR decomposition is an orthonormal basis, so it contains
    // orthogonal unit vectors.
    Eigen::Matrix<double, 3, 1> X = initial;
    Eigen::HouseholderQR<decltype(X)> qr{X};
    Eigen::Matrix<double, 3, 3> Q = qr.householderQ();

    // w = std::cos(θ/2) = std::cos(90°) = 0
    //
    // For x, y, and z, we use the second column of Q because the first is
    // parallel instead of orthogonal. The third column would also work.
    m_q = Quaternion{0.0, Q.coeff(0, 1), Q.coeff(1, 1), Q.coeff(2, 1)};
  } else {
    // initial x final
    Eigen::Vector3d axis{
        initial.coeff(1) * final.coeff(2) - final.coeff(1) * initial.coeff(2),
        final.coeff(0) * initial.coeff(2) - initial.coeff(0) * final.coeff(2),
        initial.coeff(0) * final.coeff(1) - final.coeff(0) * initial.coeff(1)};

    // https://stackoverflow.com/a/11741520
    m_q = Quaternion{normProduct + dot, axis.coeff(0), axis.coeff(1),
                     axis.coeff(2)}
              .Normalize();
  }
}

void frc::to_json(wpi::json& json, const Rotation3d& rotation) {
  json = wpi::json{{"quaternion", rotation.GetQuaternion()}};
}

void frc::from_json(const wpi::json& json, Rotation3d& rotation) {
  rotation = Rotation3d{json.at("quaternion").get<Quaternion>()};
}
