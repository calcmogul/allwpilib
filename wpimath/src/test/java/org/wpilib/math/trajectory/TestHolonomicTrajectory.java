// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.trajectory;

import static org.wpilib.math.util.Units.feetToMeters;

import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

@SuppressWarnings("PMD.TestClassWithoutTestCases")
public final class TestHolonomicTrajectory {
  private TestHolonomicTrajectory() {
    // Utility class.
  }

  public static Trajectory<HolonomicSample> getTrajectory() {
    final double maxVelocity = feetToMeters(12.0);
    final double maxAccel = feetToMeters(12);

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(feetToMeters(1.54), feetToMeters(23.23), Rotation2d.PI);
    var crossScale =
        new Pose2d(feetToMeters(23.7), feetToMeters(6.8), Rotation2d.fromDegrees(-160));

    var waypoints = new ArrayList<Pose2d>();
    waypoints.add(sideStart);
    waypoints.add(
        sideStart.plus(
            new Transform2d(
                new Translation2d(feetToMeters(-13), feetToMeters(0)), Rotation2d.ZERO)));
    waypoints.add(
        sideStart.plus(
            new Transform2d(
                new Translation2d(feetToMeters(-19.5), feetToMeters(5)), Rotation2d.CW_PI_2)));
    waypoints.add(crossScale);

    return HolonomicTrajectoryGenerator.generate(waypoints, maxVelocity, 1.0, maxAccel, 1.0);
  }
}
