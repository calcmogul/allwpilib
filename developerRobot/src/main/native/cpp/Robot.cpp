// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/trajectory/ExponentialProfile.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

class Robot : public frc::TimedRobot {
 public:
  inline static constexpr units::second_t dt = 10_ms;
  inline static constexpr units::second_t offset = 5_ms;

  inline static constexpr auto Ks = 0.1_V;
  inline static constexpr auto Kv = 3_V / 1_mps;
  inline static constexpr auto Ka = 0.5_V / 1_mps_sq;

  inline static constexpr auto Kp = 0.1_V / 1_m;
  inline static constexpr auto Kd = 0.1_V / 1_mps;

  frc::Encoder encoder{0, 1};
  frc::PWMSparkMax motor{0};

  frc::ExponentialProfile<units::meter, units::volt> profile{{10_V, Kv, Ka}};

  frc::SimpleMotorFeedforward<units::meter> feedforward{Ks, Kv, Ka};
  frc::PIDController feedback{Kp.value(), 0.0, Kd.value(), dt};

  decltype(profile)::State currentSetpoint;
  decltype(profile)::State goal;

  Robot() {
    AddPeriodic([this] { Controller(); }, dt, offset);
  }

  void SetGoal(units::meter_t goal) { this->goal.position = goal; }

  void Controller() {
    if (IsDisabled()) {
      return;
    }

    auto nextSetpoint = profile.Calculate(dt, currentSetpoint, goal);
    motor.SetVoltage(
        feedforward.Calculate(currentSetpoint.velocity, nextSetpoint.velocity,
                              dt) +
        units::volt_t{feedback.Calculate(encoder.GetDistance(),
                                         currentSetpoint.position.value())});
    currentSetpoint = nextSetpoint;
  }
};

int main() {
  return frc::StartRobot<Robot>();
}
