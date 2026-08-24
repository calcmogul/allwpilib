// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include "wpi/math/geometry/Transform2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/trajectory/HolonomicTrajectory.hpp"
#include "wpi/math/trajectory/HolonomicTrajectoryGenerator.hpp"
#include "wpi/units/acceleration.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/velocity.hpp"

namespace wpi::math {

class TestHolonomicTrajectory {
 public:
  static HolonomicTrajectory GetTrajectory(
      wpi::units::meters_per_second_t v_max,
      wpi::units::meters_per_second_squared_t a_max) {
    // 2018 cross scale auto waypoints
    const Pose2d sideStart{1.54_ft, 23.23_ft, 180_deg};
    const Pose2d crossScale{23.7_ft, 6.8_ft, -160_deg};

    auto vector = std::vector<Translation2d>{
        (sideStart + Transform2d{Translation2d{-13_ft, 0_ft}, 0_deg})
            .Translation(),
        (sideStart + Transform2d{Translation2d{-19.5_ft, 5.0_ft}, -90_deg})
            .Translation()};

    return HolonomicTrajectoryGenerator::Generate(sideStart, vector, crossScale,
                                                  v_max, 1_rad_per_s, a_max,
                                                  1_rad_per_s_sq);
  }
};

}  // namespace wpi::math
