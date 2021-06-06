// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ReplaceMeParallelDeadlineGroup.h"

#include <functional>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ReplaceMeParallelDeadlineGroup::ReplaceMeParallelDeadlineGroup()
    : CommandHelper(
          // The deadline command
          frc2::InstantCommand([] {})) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
}
