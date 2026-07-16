// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/estimator/DCMotorInputErrorEstimator.hpp"

#include <gcem.hpp>
#include <gtest/gtest.h>

#include "wpi/units/acceleration.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/math.hpp"
#include "wpi/units/time.hpp"
#include "wpi/units/velocity.hpp"
#include "wpi/units/voltage.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
  EXPECT_LE(wpi::units::math::abs(val1 - val2), eps)

TEST(DCMotorInputErrorEstimatorTest, ConstantBias) {
  constexpr auto kv = 1_V / 1_mps;
  constexpr auto ka = 0.1_V / 1_mps_sq;
  constexpr wpi::units::second_t dt = 5_ms;
  constexpr wpi::units::second_t duration = 2_s;

  // Continuous DC motor model
  constexpr auto A_c = -kv / ka;
  constexpr auto B_c = 1 / ka;

  // Discrete DC motor model
  constexpr auto A_d = wpi::units::math::exp(A_c * dt);
  constexpr auto B_d = 1 / A_c * (A_d - 1) * B_c;

  wpi::math::DCMotorInputErrorEstimator<wpi::units::meters> estimator{kv, ka,
                                                                      dt};
  wpi::units::meters_per_second_t x = 0_mps;
  wpi::units::volt_t u = 5_V;
  wpi::units::volt_t u_bias_estimate = 0_V;

  estimator.Reset(x);

  // Positive bias
  wpi::units::volt_t u_bias = 2_V;
  for (auto t = 0_s; t < duration; t += dt) {
    u_bias_estimate = estimator.Calculate(x, u);
    x = A_d * x + B_d * (u + u_bias);
  }
  EXPECT_NEAR_UNITS((u + u_bias) / kv, x, 1e-6_mps);
  EXPECT_NEAR_UNITS(u_bias, u_bias_estimate, 1e-6_V);

  // Negative bias
  u_bias = -2_V;
  for (auto t = 0_s; t < duration; t += dt) {
    u_bias_estimate = estimator.Calculate(x, u);
    x = A_d * x + B_d * (u + u_bias);
  }
  EXPECT_NEAR_UNITS((u + u_bias) / kv, x, 1e-6_mps);
  EXPECT_NEAR_UNITS(u_bias, u_bias_estimate, 1e-6_V);

  // Zero bias
  u_bias = 0_V;
  for (auto t = 0_s; t < duration; t += dt) {
    u_bias_estimate = estimator.Calculate(x, u);
    x = A_d * x + B_d * (u + u_bias);
  }
  EXPECT_NEAR_UNITS((u + u_bias) / kv, x, 1e-6_mps);
  EXPECT_NEAR_UNITS(u_bias, u_bias_estimate, 1e-6_V);
}
