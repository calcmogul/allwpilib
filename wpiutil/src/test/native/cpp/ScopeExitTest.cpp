// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "gtest/gtest.h"
#include "wpi/ScopeExit.h"

TEST(ScopeExitTest, OutOfScope) {
  int exitCount = 0;

  {
    wpi::ScopeExit exit{[&] { ++exitCount; }};

    EXPECT_EQ(0, exitCount);
  }

  EXPECT_EQ(1, exitCount);
}
