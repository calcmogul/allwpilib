/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/PeriodVariantKalmanFilterCoeffs.h>
#include <frc/controller/PeriodVariantLoop.h>
#include <frc/controller/PeriodVariantPlantCoeffs.h>
#include <frc/controller/StateSpaceControllerCoeffs.h>

frc::PeriodVariantPlantCoeffs<2, 1, 1> MakeSingleJointedArmPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 1, 1> MakeSingleJointedArmControllerCoeffs();
frc::PeriodVariantKalmanFilterCoeffs<2, 1, 1>
MakeSingleJointedArmObserverCoeffs();
frc::PeriodVariantLoop<2, 1, 1> MakeSingleJointedArmLoop();
