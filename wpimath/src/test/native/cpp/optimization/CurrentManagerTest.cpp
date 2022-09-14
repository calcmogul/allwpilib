// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/array.h>

#include "frc/CurrentManager.h"
#include "gtest/gtest.h"

TEST(CurrentManagerTest, EnoughCurrent) {
  frc::CurrentManager<4> manager{wpi::array{1_A, 5_A, 10_A, 5_A}, 40_A};

  auto currents = manager.Calculate(wpi::array{25_A, 10_A, 5_A, 0_A});

  EXPECT_DOUBLE_EQ(25.0, currents[0].value());
  EXPECT_DOUBLE_EQ(10.0, currents[1].value());
  EXPECT_DOUBLE_EQ(5.0, currents[2].value());
  EXPECT_DOUBLE_EQ(0.0, currents[3].value());
}

TEST(CurrentManagerTest, NotEnoughCurrent) {
  frc::CurrentManager<4> manager{wpi::array{1_A, 5_A, 10_A, 5_A}, 40_A};

  auto currents = manager.Calculate(wpi::array{30_A, 10_A, 5_A, 0_A});

  EXPECT_DOUBLE_EQ(29.960317463044998, currents[0].value());
  EXPECT_DOUBLE_EQ(9.007936577007202, currents[1].value());
  EXPECT_DOUBLE_EQ(1.0317463470386752, currents[2].value());
  EXPECT_DOUBLE_EQ(1.4545684970529923e-09, currents[3].value());
}
