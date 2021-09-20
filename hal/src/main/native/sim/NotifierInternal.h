// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace hal {
/**
 * Pauses all Notifiers.
 */
void PauseNotifiers();

/**
 * Resumes all Notifiers and wakes them up.
 */
void ResumeNotifiers();

/**
 * Wakes up all Notifiers.
 */
void WakeupNotifiers();

/**
 * Waits for all Notifiers to reach HAL_WaitForNotifierAlarm().
 */
void WaitNotifiers();

/**
 * Wakes up all Notifiers, then waits for all of them to reach
 * HAL_WaitForNotifierAlarm().
 */
void WakeupWaitNotifiers();
}  // namespace hal
