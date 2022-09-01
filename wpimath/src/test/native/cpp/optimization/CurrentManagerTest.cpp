// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/array.h>

#include "frc/CurrentManager.h"
#include "gtest/gtest.h"

TEST(CurrentManagerTest, CurrentManager) {
  frc::CurrentManager<4> manager{wpi::array{0.1_A, 5_A, 5_A, 5_A}, 40_A};

  auto currents = manager.Calculate(wpi::array{40_A, 10_A, 5_A, 0_A});

  EXPECT_DOUBLE_EQ(0.0, currents[0].value());
  EXPECT_DOUBLE_EQ(0.0, currents[1].value());
  EXPECT_DOUBLE_EQ(0.0, currents[2].value());
  EXPECT_DOUBLE_EQ(0.0, currents[3].value());
}
