// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

class TrajectoryTransformTest {
  @Test
  void testTransformBy() {
    var trajectory =
        HolonomicTrajectoryGenerator.generate(
            Pose2d.ZERO, List.of(), new Pose2d(1, 1, Rotation2d.CCW_PI_2), 3, 1, 3, 1);

    var transformedTrajectory =
        trajectory.transformBy(
            new Transform2d(new Translation2d(1, 2), Rotation2d.fromDegrees(30)));

    // Test initial pose.
    assertEquals(
        new Pose2d(1, 2, Rotation2d.fromDegrees(30)), transformedTrajectory.sampleAt(0).pose);

    testSameShapedTrajectory(trajectory.samples, transformedTrajectory.samples);
    testSameForwardScalars(trajectory.samples, transformedTrajectory.samples);
  }

  @Test
  void testRelativeTo() {
    var trajectory =
        HolonomicTrajectoryGenerator.generate(
            new Pose2d(1, 2, Rotation2d.fromDegrees(30.0)),
            List.of(),
            new Pose2d(5, 7, Rotation2d.CCW_PI_2),
            3,
            1,
            3,
            1);

    var transformedTrajectory = trajectory.relativeTo(new Pose2d(1, 2, Rotation2d.fromDegrees(30)));

    // Test initial pose.
    assertEquals(Pose2d.ZERO, transformedTrajectory.sampleAt(0).pose);

    testSameShapedTrajectory(trajectory.samples, transformedTrajectory.samples);
    testSameForwardScalars(trajectory.samples, transformedTrajectory.samples);
  }

  // A rigid transform rotates both the heading and the field-relative velocity/acceleration by the
  // same amount, so the heading-relative forward scalars (and curvature) are invariant. This would
  // fail if transformBy/relativeTo rotated the pose but not the velocity/acceleration.
  void testSameForwardScalars(List<HolonomicSample> statesA, List<HolonomicSample> statesB) {
    assertEquals(statesA.size(), statesB.size());
    for (int i = 0; i < statesA.size(); i++) {
      assertEquals(statesA.get(i).velocity.vx, statesB.get(i).velocity.vx, 1e-9);
      assertEquals(statesA.get(i).velocity.vy, statesB.get(i).velocity.vy, 1e-9);
      assertEquals(statesA.get(i).velocity.omega, statesB.get(i).velocity.omega, 1e-9);
      assertEquals(statesA.get(i).acceleration.ax, statesB.get(i).acceleration.ax, 1e-9);
      assertEquals(statesA.get(i).acceleration.ay, statesB.get(i).acceleration.ay, 1e-9);
      assertEquals(statesA.get(i).acceleration.alpha, statesB.get(i).acceleration.alpha, 1e-9);
    }
  }

  void testSameShapedTrajectory(List<HolonomicSample> statesA, List<HolonomicSample> statesB) {
    for (int i = 0; i < statesA.size() - 1; i++) {
      var a1 = statesA.get(i).pose;
      var a2 = statesA.get(i + 1).pose;

      var b1 = statesB.get(i).pose;
      var b2 = statesB.get(i + 1).pose;

      var expectedRel = a2.relativeTo(a1);
      var actualRel = b2.relativeTo(b1);

      assertEquals(
          expectedRel,
          actualRel,
          String.format(
              "pose mismatch at index %d: expected %s, actual %s", i, expectedRel, actualRel));
    }
  }
}
