/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeElevatorPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 1, 1> MakeElevatorControllerCoeffs();
frc::StateSpaceObserverCoeffs<2, 1, 1> MakeElevatorObserverCoeffs();
frc::StateSpaceLoop<2, 1, 1> MakeElevatorLoop();
