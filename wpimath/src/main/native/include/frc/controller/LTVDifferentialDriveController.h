// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <wpi/interpolating_map.h>

#include "frc/DARE.h"
#include "frc/EigenCore.h"
#include "frc/MathUtil.h"
#include "frc/StateSpaceUtil.h"
#include "frc/controller/DifferentialDriveWheelVoltages.h"
#include "frc/geometry/Pose2d.h"
#include "frc/system/Discretization.h"
#include "frc/system/LinearSystem.h"
#include "frc/trajectory/Trajectory.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace frc {

/**
 * The linear time-varying differential drive controller has a similar form to
 * the LQR, but the model used to compute the controller gain is the nonlinear
 * differential drive model linearized around the drivetrain's current state. We
 * precompute gains for important places in our state-space, then interpolate
 * between them with a lookup table to save computational resources.
 *
 * This controller has a flat hierarchy with pose and wheel velocity references
 * and voltage outputs. This is different from a Ramsete controller's nested
 * hierarchy where the top-level controller has a pose reference and chassis
 * velocity command outputs, and the low-level controller has wheel velocity
 * references and voltage outputs. Flat hierarchies are easier to tune in one
 * shot. Furthermore, this controller is more optimal in the "least-squares
 * error" sense than a controller based on Ramsete.
 *
 * See section 8.7 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.7.4.
 */
class WPILIB_DLLEXPORT LTVDifferentialDriveController {
 public:
  /**
   * States of the drivetrain system.
   */
  class State {
   public:
    /// X position in global coordinate frame.
    static constexpr int kX = 0;

    /// Y position in global coordinate frame.
    static constexpr int kY = 1;

    /// Heading in global coordinate frame.
    static constexpr int kHeading = 2;

    /// Left encoder velocity.
    static constexpr int kLeftVelocity = 3;

    /// Right encoder velocity.
    static constexpr int kRightVelocity = 4;
  };

  /**
   * Constructs a linear time-varying differential drive controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param plant      The differential drive velocity plant.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param Qelems     The maximum desired error tolerance for each state.
   * @param Relems     The maximum desired control effort for each input.
   * @param dt         Discretization timestep.
   * @throws std::domain_error if max velocity of plant with 12 V input <= 0 m/s
   *     or >= 15 m/s.
   */
  constexpr LTVDifferentialDriveController(
      const frc::LinearSystem<2, 2, 2>& plant, units::meter_t trackwidth,
      const wpi::array<double, 5>& Qelems, const wpi::array<double, 2>& Relems,
      units::second_t dt)
      : m_trackwidth{trackwidth} {
    // Control law derivation is in section 8.7 of
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    Matrixd<5, 5> A{{0.0, 0.0, 0.0, 0.5, 0.5},
                    {0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, -1.0 / m_trackwidth.value(),
                     1.0 / m_trackwidth.value()},
                    {0.0, 0.0, 0.0, plant.A(0, 0), plant.A(0, 1)},
                    {0.0, 0.0, 0.0, plant.A(1, 0), plant.A(1, 1)}};
    Matrixd<5, 2> B{{0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {plant.B(0, 0), plant.B(0, 1)},
                    {plant.B(1, 0), plant.B(1, 1)}};
    Matrixd<5, 5> Q = frc::MakeCostMatrix(Qelems);
    Matrixd<2, 2> R = frc::MakeCostMatrix(Relems);

    // dx/dt = Ax + Bu
    // 0 = Ax + Bu
    // Ax = -Bu
    // x = -A⁻¹Bu
    units::meters_per_second_t maxV{
        // NOLINTNEXTLINE(clang-analyzer-unix.Malloc)
        -plant.A().householderQr().solve(plant.B() *
                                         Vectord<2>{12.0, 12.0})(0)};

    if (maxV <= 0_mps) {
      throw std::domain_error(
          "Max velocity of plant with 12 V input must be greater than 0 m/s.");
    }
    if (maxV >= 15_mps) {
      throw std::domain_error(
          "Max velocity of plant with 12 V input must be less than 15 m/s.");
    }

    auto R_llt = R.llt();

    for (auto velocity = -maxV; velocity < maxV; velocity += 0.01_mps) {
      // The DARE is ill-conditioned if the velocity is close to zero, so don't
      // let the system stop.
      if (units::math::abs(velocity) < 1e-4_mps) {
        A(State::kY, State::kHeading) = 1e-4;
      } else {
        A(State::kY, State::kHeading) = velocity.value();
      }

      Matrixd<5, 5> discA;
      Matrixd<5, 2> discB;
      DiscretizeAB(A, B, dt, &discA, &discB);

      auto S = detail::DARE<5, 2>(discA, discB, Q, R_llt);

      // K = (BᵀSB + R)⁻¹BᵀSA
      m_table.insert(velocity, (discB.transpose() * S * discB + R)
                                   .llt()
                                   .solve(discB.transpose() * S * discA));
    }
  }

  /**
   * Move constructor.
   */
  constexpr LTVDifferentialDriveController(LTVDifferentialDriveController&&) =
      default;

  /**
   * Move assignment operator.
   */
  constexpr LTVDifferentialDriveController& operator=(
      LTVDifferentialDriveController&&) = default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  constexpr bool AtReference() const {
    return std::abs(m_error(0)) < m_tolerance(0) &&
           std::abs(m_error(1)) < m_tolerance(1) &&
           std::abs(m_error(2)) < m_tolerance(2) &&
           std::abs(m_error(3)) < m_tolerance(3) &&
           std::abs(m_error(4)) < m_tolerance(4);
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   * @param leftVelocityTolerance Left velocity error which is tolerable.
   * @param rightVelocityTolerance Right velocity error which is tolerable.
   */
  constexpr void SetTolerance(
      const Pose2d& poseTolerance,
      units::meters_per_second_t leftVelocityTolerance,
      units::meters_per_second_t rightVelocityTolerance) {
    m_tolerance = Vectord<5>{
        poseTolerance.X().value(), poseTolerance.Y().value(),
        poseTolerance.Rotation().Radians().value(),
        leftVelocityTolerance.value(), rightVelocityTolerance.value()};
  }

  /**
   * Returns the left and right output voltages of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose      The current pose.
   * @param leftVelocity     The current left velocity.
   * @param rightVelocity    The current right velocity.
   * @param poseRef          The desired pose.
   * @param leftVelocityRef  The desired left velocity.
   * @param rightVelocityRef The desired right velocity.
   */
  constexpr DifferentialDriveWheelVoltages Calculate(
      const Pose2d& currentPose, units::meters_per_second_t leftVelocity,
      units::meters_per_second_t rightVelocity, const Pose2d& poseRef,
      units::meters_per_second_t leftVelocityRef,
      units::meters_per_second_t rightVelocityRef) {
    // This implements the linear time-varying differential drive controller in
    // theorem 9.6.3 of https://tavsys.net/controls-in-frc.
    Vectord<5> x{currentPose.X().value(), currentPose.Y().value(),
                 currentPose.Rotation().Radians().value(), leftVelocity.value(),
                 rightVelocity.value()};

    Matrixd<5, 5> inRobotFrame = Matrixd<5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(State::kHeading));
    inRobotFrame(0, 1) = std::sin(x(State::kHeading));
    inRobotFrame(1, 0) = -std::sin(x(State::kHeading));
    inRobotFrame(1, 1) = std::cos(x(State::kHeading));

    Vectord<5> r{poseRef.X().value(), poseRef.Y().value(),
                 poseRef.Rotation().Radians().value(), leftVelocityRef.value(),
                 rightVelocityRef.value()};
    m_error = r - x;
    m_error(State::kHeading) =
        frc::AngleModulus(units::radian_t{m_error(State::kHeading)}).value();

    units::meters_per_second_t velocity{(leftVelocity + rightVelocity) / 2.0};
    const auto& K = m_table[velocity];

    Vectord<2> u = K * inRobotFrame * m_error;

    return DifferentialDriveWheelVoltages{units::volt_t{u(0)},
                                          units::volt_t{u(1)}};
  }

  /**
   * Returns the left and right output voltages of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose  The current pose.
   * @param leftVelocity The left velocity.
   * @param rightVelocity The right velocity.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   */
  constexpr DifferentialDriveWheelVoltages Calculate(
      const Pose2d& currentPose, units::meters_per_second_t leftVelocity,
      units::meters_per_second_t rightVelocity,
      const Trajectory::State& desiredState) {
    // v = (v_r + v_l) / 2     (1)
    // w = (v_r - v_l) / (2r)  (2)
    // k = w / v               (3)
    //
    // v_l = v - wr
    // v_l = v - (vk)r
    // v_l = v(1 - kr)
    //
    // v_r = v + wr
    // v_r = v + (vk)r
    // v_r = v(1 + kr)
    return Calculate(
        currentPose, leftVelocity, rightVelocity, desiredState.pose,
        desiredState.velocity *
            (1 - (desiredState.curvature / 1_rad * m_trackwidth / 2.0)),
        desiredState.velocity *
            (1 + (desiredState.curvature / 1_rad * m_trackwidth / 2.0)));
  }

 private:
  units::meter_t m_trackwidth;

  // LUT from drivetrain linear velocity to LQR gain
  wpi::interpolating_map<units::meters_per_second_t, Matrixd<2, 5>> m_table;

  Vectord<5> m_error;
  Vectord<5> m_tolerance;
};

}  // namespace frc
