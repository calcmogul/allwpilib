// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/geometry/Ellipse2d.hpp"

#include <cmath>

#include <sleipnir/optimization/solver/sqp.hpp>

using namespace wpi::math;

Translation2d Ellipse2d::Nearest(const Translation2d& point) const {
  // Check if already in ellipse
  if (Contains(point)) {
    return point;
  }

  // Rotate the point by the inverse of the ellipse's rotation
  auto rotPoint =
      point.RotateAround(m_center.Translation(), -m_center.Rotation());

  // Find nearest point
  {
    // min (x − x_0)² + (y − y_0)²
    // x,y
    // s.t. (x − x_c)²/a² + (y − y_c)²/b² = 1
    const double x_0 = rotPoint.X().value();
    const double y_0 = rotPoint.Y().value();
    const double x_c = m_center.X().value();
    const double y_c = m_center.Y().value();
    const double a2 = m_xSemiAxis.value() * m_xSemiAxis.value();
    const double b2 = m_ySemiAxis.value() * m_ySemiAxis.value();

    Eigen::VectorXd x{{x_0}, {y_0}};
    slp::sqp(
        slp::SQPMatrixCallbacks<double>{
            [&](const Eigen::VectorXd& x) -> double {
              // f(x) = (x[0] − x_0)² + (x[1] − y_0)²
              return std::pow(x[0] - x_0, 2) + std::pow(x[1] - y_0, 2);
            },
            [&](const Eigen::VectorXd& x) -> Eigen::SparseVector<double> {
              // ∇f(x) = [2(x[0] − x_0)]
              //         [2(x[1] − y_0)]
              return Eigen::Vector2d{{2.0 * (x[0] - x_0)}, {2.0 * (x[1] - y_0)}}
                  .sparseView();
            },
            [&](const Eigen::VectorXd& x,
                const Eigen::VectorXd& y) -> Eigen::SparseMatrix<double> {
              // L(x, y) = f(x) − yᵀcₑ(x)
              //
              // ∇ₓₓ²L(x, y) = [2 − 2b²y     0    ]
              //               [   0      2 − 2a²y]
              return Eigen::SparseMatrix<double>{
                  Eigen::DiagonalMatrix<double, 2>{2.0 - 2.0 * b2 * y[0],
                                                   2.0 - 2.0 * a2 * y[0]}};
            },
            [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
              // (x − x_c)²/a² + (y − y_c)²/b² = 1
              // b²(x − x_c)² + a²(y − y_c)² = a²b²
              // b²(x − x_c)² + a²(y − y_c)² − a²b² = 0
              //
              // cₑ(x) = b²(x − x_c)² + a²(y − y_c)² − a²b²
              return Eigen::Vector<double, 1>{{b2 * std::pow(x[0] - x_c, 2) +
                                               a2 * std::pow(x[1] - y_c, 2) -
                                               a2 * b2}}
                  .sparseView();
            },
            [&](const Eigen::VectorXd& x) -> Eigen::SparseMatrix<double> {
              // Aₑ(x) = [2b²(x[0] − x_c)  2a²(x[1] − y_c)]
              return Eigen::RowVector2d{
                  {2.0 * b2 * (x[0] - x_c), 2.0 * a2 * (x[1] - y_c)}}
                  .sparseView();
            }},
        {}, {}, x);

    rotPoint =
        wpi::math::Translation2d{units::meter_t{x[0]}, units::meter_t{x[1]}};
  }

  // Undo rotation
  return rotPoint.RotateAround(m_center.Translation(), m_center.Rotation());
}
