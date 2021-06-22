// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

#include "frc/PIDController.h"

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <wpi/sendable/SendableBuilder.h>

#include "frc/Notifier.h"
#include "frc/PIDOutput.h"

using namespace frc;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

PIDController::PIDController(double Kp, double Ki, double Kd, PIDSource* source,
                             PIDOutput* output, double period)
    : PIDController(Kp, Ki, Kd, 0.0, *source, *output, period) {}

PIDController::PIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource* source, PIDOutput* output,
                             double period)
    : PIDController(Kp, Ki, Kd, Kf, *source, *output, period) {}

PIDController::PIDController(double Kp, double Ki, double Kd, PIDSource& source,
                             PIDOutput& output, double period)
    : PIDController(Kp, Ki, Kd, 0.0, source, output, period) {}

PIDController::PIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource& source, PIDOutput& output,
                             double period)
    : PIDBase(Kp, Ki, Kd, Kf, source, output) {
  m_controlLoop = std::make_unique<Notifier>(&PIDController::Calculate, this);
  m_controlLoop->StartPeriodic(units::second_t(period));
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

PIDController::~PIDController() {
  // Forcefully stopping the notifier so the callback can successfully run.
  m_controlLoop->Stop();
}

void PIDController::Enable() {
  {
    std::scoped_lock lock(m_thisMutex);
    m_enabled = true;
  }
}

void PIDController::Disable() {
  {
    // Ensures m_enabled modification and PIDWrite() call occur atomically
    std::scoped_lock pidWriteLock(m_pidWriteMutex);
    {
      std::scoped_lock mainLock(m_thisMutex);
      m_enabled = false;
    }

    m_pidOutput->PIDWrite(0);
  }
}

void PIDController::SetEnabled(bool enable) {
  if (enable) {
    Enable();
  } else {
    Disable();
  }
}

bool PIDController::IsEnabled() const {
  std::scoped_lock lock(m_thisMutex);
  return m_enabled;
}

void PIDController::Reset() {
  Disable();

  PIDBase::Reset();
}

void PIDController::InitSendable(wpi::SendableBuilder& builder) {
  PIDBase::InitSendable(builder);
  builder.AddBooleanProperty(
      "enabled", [=] { return IsEnabled(); },
      [=](bool value) { SetEnabled(value); });
}
