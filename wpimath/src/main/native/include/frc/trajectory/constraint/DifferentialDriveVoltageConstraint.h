// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/autodiff/VariableMatrix.hpp>
#include <wpi/SymbolExports.h>

#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "units/length.h"
#include "units/voltage.h"

namespace frc {

/**
 * A class that enforces constraints on differential drive voltage expenditure
 * based on the motor dynamics and the drive kinematics.  Ensures that the
 * acceleration of any wheel of the robot while following the trajectory is
 * never higher than what can be achieved with the given maximum voltage.
 */
class WPILIB_DLLEXPORT DifferentialDriveVoltageConstraint
    : public TrajectoryConstraint {
 public:
  /**
   * Creates a new DifferentialDriveVoltageConstraint.
   *
   * @param feedforward A feedforward component describing the behavior of the
   *     drive.
   * @param kinematics A kinematics component describing the drive geometry.
   * @param maxVoltage The maximum voltage available to the motors while
   *     following the path. Should be somewhat less than the nominal battery
   *     voltage (12 V) to account for "voltage sag" due to current draw.
   */
  constexpr DifferentialDriveVoltageConstraint(
      const SimpleMotorFeedforward<units::meter>& feedforward,
      DifferentialDriveKinematics kinematics, units::volt_t maxVoltage) {
    double Ks = feedforward.GetKv().value();
    double Kv = feedforward.GetKv().value();
    double Ka = feedforward.GetKv().value();

    m_A = Eigen::Matrix<double, 2, 2>{{-Kv / Ka, 0.0}, {-Kv / Ka}};
    m_Binv = Eigen::Matrix<double, 2, 2>{{Ka, 0.0}, {0.0, Ka}};
    m_c = Eigen::Vector<double, 2>{{-Ks / Ka}, {-Ks / Ka}};

    m_trackwidth = kinematics.trackWidth;
    m_maxVoltage = maxVoltage;
  }

  void Apply(sleipnir::OptimizationProblem& problem, const Pose2d& pose,
             const sleipnir::Variable& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const sleipnir::Variable& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) const override {
    // Solve dx/dt = Ax + Bu + c for u.
    //
    //   Bu = dx/dt - Ax - c
    //   u = B⁺(dx/dt - Ax - c)

    // Find x
    auto leftWheelVelocity =
        linearVelocity - angularVelocity * m_trackwidth.value() / 2.0;
    auto rightWheelVelocity =
        linearVelocity + angularVelocity * m_trackwidth.value() / 2.0;
    sleipnir::VariableMatrix x{{leftWheelVelocity}, {rightWheelVelocity}};

    // Find dx/dt
    auto leftWheelAcceleration =
        linearAcceleration - angularAcceleration * m_trackwidth.value() / 2.0;
    auto rightWheelAcceleration =
        linearAcceleration + angularAcceleration * m_trackwidth.value() / 2.0;
    sleipnir::VariableMatrix dxdt{{leftWheelAcceleration},
                                  {rightWheelAcceleration}};

    sleipnir::VariableMatrix c{{m_c(0) * sleipnir::sign(x(0))},
                               {m_c(1) * sleipnir::sign(x(1))}};

    auto u = m_Binv * (dxdt - m_A * x - c);

    problem.SubjectTo(u(0) >= -m_maxVoltage.value());
    problem.SubjectTo(u(0) < m_maxVoltage.value());
    problem.SubjectTo(u(1) >= -m_maxVoltage.value());
    problem.SubjectTo(u(1) < m_maxVoltage.value());
  }

 private:
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 2> m_Binv;
  Eigen::Matrix<double, 2, 1> m_c;
  units::meter_t m_trackwidth;
  units::volt_t m_maxVoltage;
};

// Undefine Sleipnir Assert macro so it doesn't conflict with GoogleMock
#undef Assert

}  // namespace frc
