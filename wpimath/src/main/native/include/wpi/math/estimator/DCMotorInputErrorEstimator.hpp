// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>

#include "wpi/math/estimator/SteadyStateKalmanFilter.hpp"
#include "wpi/math/system/LinearSystem.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/base.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/units/voltage.hpp"

namespace wpi::math {

/**
 * This class estimates the input error required to make the expected motor
 * velocity match the actual velocity over one timestep.
 *
 * Subtracting this input error from the next control input will correct for the
 * bias and eliminate steady-state error. This method of fixing steady-state
 * error is superior to a pure integral controller because it only integrates
 * for unmodeled disturbances, not during expected transient behavior while
 * converging to a reference. This means there won't be overshoot during
 * transients.
 */
template <class Distance>
  requires wpi::units::length_unit<Distance> ||
           wpi::units::angle_unit<Distance> ||
           wpi::units::dimensionless_unit<Distance>
class DCMotorInputErrorEstimator {
 public:
  using Velocity =
      wpi::units::compound_unit<Distance, wpi::units::inverse<units::seconds>>;
  using Velocity_t = wpi::units::unit_t<Velocity>;
  using Acceleration =
      wpi::units::compound_unit<Velocity, wpi::units::inverse<units::seconds>>;

  using Kv_t = wpi::units::unit_t<
      wpi::units::compound_unit<units::volt, wpi::units::inverse<Velocity>>>;
  using Ka_t = wpi::units::unit_t<wpi::units::compound_unit<
      units::volt, wpi::units::inverse<Acceleration>>>;

  /**
   * Constructs a DC motor input error estimator.
   *
   * @param Kv The velocity feedforward gain.
   * @param Ka The acceleration feedforward gain.
   * @param dt Nominal discretization timestep.
   */
  DCMotorInputErrorEstimator(Kv_t Kv, Ka_t Ka, wpi::units::second_t dt)
      : m_dt{dt},
        m_system{Eigen::Matrix<double, 2, 2>{
                     {-Kv.value() / Ka.value(), 1 / Ka.value()}, {0, 0}},
                 Eigen::Matrix<double, 2, 1>{{1 / Ka.value()}, {0}},
                 Eigen::Matrix<double, 1, 2>{{1, 0}},
                 Eigen::Matrix<double, 1, 1>{0}},
        // TODO: Find velocity model standard deviation and velocity measurement
        // standard deviation dynamically
        m_kf{m_system, {0.1, 12.0}, {0.1}, m_dt} {}

  /**
   * Returns the input error estimate.
   *
   * @param velocity The velocity from the current timestep.
   * @param voltage The voltage applied for the current timestep.
   */
  wpi::units::volt_t Calculate(Velocity_t velocity, units::volt_t voltage) {
    // NOTE: Correct() should be called with the voltage from the previous
    // timestep instead of the current timestep, but it doesn't affect accuracy
    // because the system's D matrix is zero.
    m_kf.Correct(Eigen::Vector<double, 1>{voltage.value()},
                 Eigen::Vector<double, 1>{velocity.value()});
    m_kf.Predict(Eigen::Vector<double, 1>{voltage.value()}, m_dt);

    return wpi::units::volt_t{m_kf.Xhat(1)};
  }

  /**
   * Resets the estimator to the given velocity with no input error.
   *
   * @param velocity The current velocity.
   */
  void Reset(Velocity_t velocity) {
    m_kf.SetXhat(Eigen::Vector<double, 2>{velocity.value(), 0.0});
  }

 private:
  wpi::units::second_t m_dt;
  LinearSystem<2, 1, 1> m_system;
  SteadyStateKalmanFilter<2, 1, 1> m_kf;
};

}  // namespace wpi::math
