/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/controller/RamseteController.h"

#include <cmath>

using namespace frc;

/**
 * Returns sin(x) / x.
 *
 * @param x Value of which to take sinc(x).
 */
static double Sinc(double x) {
  if (std::abs(x) < 1e-9) {
    return 1.0;
  } else {
    return std::sin(x) / x;
  }
}

RamseteController::RamseteController(double b, double zeta)
    : m_b{b}, m_zeta{zeta} {}

RamseteController::Outputs RamseteController::Calculate(
    const Pose2d& desiredPose, units::meters_per_second_t desiredLinearVelocity,
    units::radians_per_second_t desiredAngularVelocity,
    const Pose2d& currentPose) {
  using units::unit_cast;

  auto error = desiredPose.RelativeTo(currentPose);

  // Aliases for equation readability
  auto vDesired = unit_cast<double>(desiredLinearVelocity);
  auto omegaDesired = unit_cast<double>(desiredAngularVelocity);
  double eX = error.Translation().X();
  double eY = error.Translation().Y();
  double eTheta = error.Rotation().Radians();

  double k = 2.0 * m_zeta *
             std::sqrt(std::pow(omegaDesired, 2) + m_b * std::pow(vDesired, 2));

  units::meters_per_second_t v{vDesired * std::cos(eTheta) + k * eX};
  units::radians_per_second_t omega{omegaDesired + k * eTheta +
                                    m_b * vDesired * Sinc(eTheta) * eY};

  return Outputs{v, omega};
}
