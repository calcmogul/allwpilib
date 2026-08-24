// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/estimator/KalmanFilter.hpp"

#include <vector>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "wpi/math/TestAssertions.hpp"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/random/Normal.hpp"
#include "wpi/math/system/LinearSystem.hpp"
#include "wpi/math/trajectory/HolonomicTrajectoryGenerator.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/units/velocity.hpp"

TEST_CASE("KalmanFilterTest SwerveStationary", "[wpimath]") {
  constexpr wpi::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  wpi::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  wpi::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {1.0, 1.0, 1.0}, dt};

  constexpr Eigen::Vector<double, 3> u{{0.0}, {0.0}, {0.0}};

  for (int i = 0; i < 100; ++i) {
    auto y = wpi::math::Normal(1.0, 1.0, 1.0);

    filter.Correct(u, y);
    filter.Predict(u, dt);
  }

  CHECK_NEAR(0.0, filter.Xhat(0), 0.3);
  CHECK_NEAR(0.0, filter.Xhat(1), 0.3);
}

TEST_CASE("KalmanFilterTest SwerveBadInitialPose", "[wpimath]") {
  constexpr wpi::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  wpi::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  wpi::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {0.1, 0.1, 0.25}, dt};

  // Set nonzero position
  filter.SetXhat(0, 0.5);
  filter.SetXhat(1, 0.5);

  constexpr Eigen::Vector<double, 3> u{{0.0}, {0.0}, {0.0}};

  // Let filter converge to zero position
  for (int i = 0; i < 300; ++i) {
    auto y = wpi::math::Normal(0.1, 0.1, 0.25);

    filter.Correct(u, y);
    filter.Predict(u, dt);
  }

  CHECK_NEAR(0.0, filter.Xhat(0), 0.2);
  CHECK_NEAR(0.0, filter.Xhat(1), 0.2);
}

TEST_CASE("KalmanFilterTest SwerveMovingOverTrajectory", "[wpimath]") {
  constexpr wpi::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  wpi::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  wpi::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {0.2, 0.2, 1.0 / 3.0}, dt};

  auto trajectory = wpi::math::HolonomicTrajectoryGenerator::Generate(
      std::vector{wpi::math::Pose2d{0_m, 0_m, 0_rad},
                  wpi::math::Pose2d{5_m, 5_m, 0_rad}},
      2_mps, 1_rad_per_s, 2_mps_sq, 1_rad_per_s_sq);

  Eigen::Vector3d lastVelocity{0.0, 0.0, 0.0};

  for (wpi::units::second_t t = 0_s; t < trajectory.Duration(); t += dt) {
    auto sample = trajectory.SampleAt(t);

    Eigen::Vector3d y{sample.pose.Translation().X().value(),
                      sample.pose.Translation().Y().value(),
                      sample.pose.Rotation().Radians().value()};
    y += wpi::math::Normal(0.2, 0.2, 1.0 / 3.0);

    // The velocity is field-relative; the input is its time derivative.
    Eigen::Vector3d velocity{sample.velocity.vx.value(),
                             sample.velocity.vy.value(),
                             sample.velocity.omega.value()};
    Eigen::Vector3d u = (velocity - lastVelocity) / dt.value();

    filter.Correct(u, y);
    filter.Predict(u, dt);

    lastVelocity = velocity;
  }

  CHECK_NEAR(
      trajectory.SampleAt(trajectory.Duration()).pose.Translation().X().value(),
      filter.Xhat(0), 0.2);
  CHECK_NEAR(
      trajectory.SampleAt(trajectory.Duration()).pose.Translation().Y().value(),
      filter.Xhat(1), 0.2);
}
