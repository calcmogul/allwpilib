// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <Eigen/Core>
#include <gcem.hpp>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>

#include "frc/DARE.h"
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
 * and voltage outputs. This is different from a unicycle controller's nested
 * hierarchy where the top-level controller has a pose reference and chassis
 * velocity command outputs, and the low-level controller has wheel velocity
 * references and voltage outputs. Flat hierarchies are easier to tune in one
 * shot.
 *
 * See section 8.7 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.7.4.
 */
class WPILIB_DLLEXPORT LTVDifferentialDriveController {
 public:
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
   */
  constexpr LTVDifferentialDriveController(
      const frc::LinearSystem<2, 2, 2>& plant, units::meter_t trackwidth,
      const wpi::array<double, 5>& Qelems, const wpi::array<double, 2>& Relems,
      units::second_t dt)
      : m_trackwidth{trackwidth},
        m_A{plant.A()},
        m_B{plant.B()},
        m_Q{frc::MakeCostMatrix(Qelems)},
        m_R{frc::MakeCostMatrix(Relems)},
        m_dt{dt} {}

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
    return gcem::abs(m_error(0)) < m_tolerance(0) &&
           gcem::abs(m_error(1)) < m_tolerance(1) &&
           gcem::abs(m_error(2)) < m_tolerance(2) &&
           gcem::abs(m_error(3)) < m_tolerance(3) &&
           gcem::abs(m_error(4)) < m_tolerance(4);
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
    m_tolerance = Eigen::Vector<double, 5>{
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
    // theorem 8.7.4 of https://controls-in-frc.link/
    //
    //     [x ]
    //     [y ]       [Vₗ]
    // x = [θ ]   u = [Vᵣ]
    //     [vₗ]
    //     [vᵣ]

    units::meters_per_second_t velocity{(leftVelocity + rightVelocity) / 2.0};

    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    if (units::math::abs(velocity) < 1e-4_mps) {
      velocity = 1e-4_mps;
    }

    Eigen::Vector<double, 5> r{poseRef.X().value(), poseRef.Y().value(),
                               poseRef.Rotation().Radians().value(),
                               leftVelocityRef.value(),
                               rightVelocityRef.value()};
    Eigen::Vector<double, 5> x{currentPose.X().value(), currentPose.Y().value(),
                               currentPose.Rotation().Radians().value(),
                               leftVelocity.value(), rightVelocity.value()};

    m_error = r - x;
    m_error(2) = frc::AngleModulus(units::radian_t{m_error(2)}).value();

    Eigen::Matrix<double, 5, 5> A{{0.0, 0.0, 0.0, 0.5, 0.5},
                                  {0.0, 0.0, velocity.value(), 0.0, 0.0},
                                  {0.0, 0.0, 0.0, -1.0 / m_trackwidth.value(),
                                   1.0 / m_trackwidth.value()},
                                  {0.0, 0.0, 0.0, m_A(0, 0), m_A(0, 1)},
                                  {0.0, 0.0, 0.0, m_A(1, 0), m_A(1, 1)}};
    Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                  {0.0, 0.0},
                                  {0.0, 0.0},
                                  {m_B(0, 0), m_B(0, 1)},
                                  {m_B(1, 0), m_B(1, 1)}};

    Eigen::Matrix<double, 5, 5> discA;
    Eigen::Matrix<double, 5, 2> discB;
    DiscretizeAB(A, B, m_dt, &discA, &discB);

    auto S = DARE<5, 2>(discA, discB, m_Q, m_R, false).value();

    // K = (BᵀSB + R)⁻¹BᵀSA
    Eigen::Matrix<double, 2, 5> K = (discB.transpose() * S * discB + m_R)
                                        .llt()
                                        .solve(discB.transpose() * S * discA);

    Eigen::Matrix<double, 5, 5> inRobotFrame{
        {gcem::cos(x(2)), gcem::sin(x(2)), 0.0, 0.0, 0.0},
        {-gcem::sin(x(2)), gcem::cos(x(2)), 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0}};

    Eigen::Vector2d u = K * inRobotFrame * m_error;

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

  // Continuous velocity dynamics
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 2> m_B;

  // LQR cost matrices
  Eigen::Matrix<double, 5, 5> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  units::second_t m_dt;

  Eigen::Vector<double, 5> m_error;
  Eigen::Vector<double, 5> m_tolerance;
};

}  // namespace frc
