// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include <Eigen/Cholesky>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <wpi/interpolating_map.h>

#include "frc/DARE.h"
#include "frc/EigenCore.h"
#include "frc/StateSpaceUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/system/Discretization.h"
#include "frc/trajectory/Trajectory.h"
#include "units/angular_velocity.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"

namespace frc {

/**
 * The linear time-varying unicycle controller has a similar form to the LQR,
 * but the model used to compute the controller gain is the nonlinear unicycle
 * model linearized around the drivetrain's current state.
 *
 * This controller is a roughly drop-in replacement for RamseteController with
 * more optimal feedback gains in the "least-squares error" sense.
 *
 * See section 8.9 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.9.1.
 */
class WPILIB_DLLEXPORT LTVUnicycleController {
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
  };

  /**
   * Constructs a linear time-varying unicycle controller with default maximum
   * desired error tolerances of (0.0625 m, 0.125 m, 2 rad) and default maximum
   * desired control effort of (1 m/s, 2 rad/s).
   *
   * @param dt Discretization timestep.
   * @param maxVelocity The maximum velocity for the controller gain lookup
   *                    table.
   * @throws std::domain_error if maxVelocity &lt;= 0.
   */
  constexpr explicit LTVUnicycleController(
      units::second_t dt, units::meters_per_second_t maxVelocity = 9_mps)
      : LTVUnicycleController{
            {0.0625, 0.125, 2.0}, {1.0, 2.0}, dt, maxVelocity} {}

  /**
   * Constructs a linear time-varying unicycle controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param Qelems The maximum desired error tolerance for each state.
   * @param Relems The maximum desired control effort for each input.
   * @param dt     Discretization timestep.
   * @param maxVelocity The maximum velocity for the controller gain lookup
   *                    table.
   * @throws std::domain_error if maxVelocity <= 0 m/s or >= 15 m/s.
   */
  constexpr LTVUnicycleController(
      const wpi::array<double, 3>& Qelems, const wpi::array<double, 2>& Relems,
      units::second_t dt, units::meters_per_second_t maxVelocity = 9_mps) {
    if (maxVelocity <= 0_mps) {
      throw std::domain_error("Max velocity must be greater than 0 m/s.");
    }
    if (maxVelocity >= 15_mps) {
      throw std::domain_error("Max velocity must be less than 15 m/s.");
    }

    // The change in global pose for a unicycle is defined by the following
    // three equations.
    //
    // ẋ = v cosθ
    // ẏ = v sinθ
    // θ̇ = ω
    //
    // Here's the model as a vector function where x = [x  y  θ]ᵀ and u = [v
    // ω]ᵀ.
    //
    //           [v cosθ]
    // f(x, u) = [v sinθ]
    //           [  ω   ]
    //
    // To create an LQR, we need to linearize this.
    //
    //               [0  0  −v sinθ]                  [cosθ  0]
    // ∂f(x, u)/∂x = [0  0   v cosθ]    ∂f(x, u)/∂u = [sinθ  0]
    //               [0  0     0   ]                  [ 0    1]
    //
    // We're going to make a cross-track error controller, so we'll apply a
    // clockwise rotation matrix to the global tracking error to transform it
    // into the robot's coordinate frame. Since the cross-track error is always
    // measured from the robot's coordinate frame, the model used to compute the
    // LQR should be linearized around θ = 0 at all times.
    //
    //     [0  0  −v sin0]        [cos0  0]
    // A = [0  0   v cos0]    B = [sin0  0]
    //     [0  0     0   ]        [ 0    1]
    //
    //     [0  0  0]              [1  0]
    // A = [0  0  v]          B = [0  0]
    //     [0  0  0]              [0  1]
    Matrixd<3, 3> A = Matrixd<3, 3>::Zero();
    Matrixd<3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};
    Matrixd<3, 3> Q = frc::MakeCostMatrix(Qelems);
    Matrixd<2, 2> R = frc::MakeCostMatrix(Relems);

    auto R_llt = R.llt();

    for (auto velocity = -maxVelocity; velocity < maxVelocity;
         velocity += 0.01_mps) {
      // The DARE is ill-conditioned if the velocity is close to zero, so don't
      // let the system stop.
      if (units::math::abs(velocity) < 1e-4_mps) {
        A(State::kY, State::kHeading) = 1e-4;
      } else {
        A(State::kY, State::kHeading) = velocity.value();
      }

      Matrixd<3, 3> discA;
      Matrixd<3, 2> discB;
      DiscretizeAB(A, B, dt, &discA, &discB);

      auto S = detail::DARE<3, 2>(discA, discB, Q, R_llt);

      // K = (BᵀSB + R)⁻¹BᵀSA
      m_table.insert(velocity, (discB.transpose() * S * discB + R)
                                   .llt()
                                   .solve(discB.transpose() * S * discA));
    }
  }

  /**
   * Move constructor.
   */
  constexpr LTVUnicycleController(LTVUnicycleController&&) = default;

  /**
   * Move assignment operator.
   */
  constexpr LTVUnicycleController& operator=(LTVUnicycleController&&) = default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  constexpr bool AtReference() const {
    const auto& eTranslate = m_poseError.Translation();
    const auto& eRotate = m_poseError.Rotation();
    const auto& tolTranslate = m_poseTolerance.Translation();
    const auto& tolRotate = m_poseTolerance.Rotation();
    return units::math::abs(eTranslate.X()) < tolTranslate.X() &&
           units::math::abs(eTranslate.Y()) < tolTranslate.Y() &&
           units::math::abs(eRotate.Radians()) < tolRotate.Radians();
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   */
  constexpr void SetTolerance(const Pose2d& poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  /**
   * Returns the linear and angular velocity outputs of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose        The current pose.
   * @param poseRef            The desired pose.
   * @param linearVelocityRef  The desired linear velocity.
   * @param angularVelocityRef The desired angular velocity.
   */
  constexpr ChassisSpeeds Calculate(
      const Pose2d& currentPose, const Pose2d& poseRef,
      units::meters_per_second_t linearVelocityRef,
      units::radians_per_second_t angularVelocityRef) {
    if (!m_enabled) {
      return ChassisSpeeds{linearVelocityRef, 0_mps, angularVelocityRef};
    }

    m_poseError = poseRef.RelativeTo(currentPose);

    const auto& K = m_table[linearVelocityRef];
    Vectord<3> e{m_poseError.X().value(), m_poseError.Y().value(),
                 m_poseError.Rotation().Radians().value()};
    Vectord<2> u = K * e;

    return ChassisSpeeds{
        linearVelocityRef + units::meters_per_second_t{u(0)}, 0_mps,
        angularVelocityRef + units::radians_per_second_t{u(1)}};
  }

  /**
   * Returns the linear and angular velocity outputs of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose  The current pose.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   */
  constexpr ChassisSpeeds Calculate(const Pose2d& currentPose,
                                    const Trajectory::State& desiredState) {
    return Calculate(currentPose, desiredState.pose, desiredState.velocity,
                     desiredState.velocity * desiredState.curvature);
  }

  /**
   * Enables and disables the controller for troubleshooting purposes.
   *
   * @param enabled If the controller is enabled or not.
   */
  constexpr void SetEnabled(bool enabled) { m_enabled = enabled; }

 private:
  // LUT from drivetrain linear velocity to LQR gain
  wpi::interpolating_map<units::meters_per_second_t, Matrixd<2, 3>> m_table;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
  bool m_enabled = true;
};

}  // namespace frc
