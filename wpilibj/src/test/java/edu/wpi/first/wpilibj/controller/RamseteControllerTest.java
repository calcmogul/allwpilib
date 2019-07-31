/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class RamseteControllerTest {
  private final double[][] m_trajectory = SampleTrajectory.getInstance().getPath();
  private static final double kTolerance = 1 / 12.0;
  private static final double kAngularTolerance = Math.toRadians(2);

  @Test
  @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
  void testReachesGoal() {
    final var controller = new RamseteController(2.0, 0.7);

    var robotPose = new Pose2d(2.7, 23.0, Rotation2d.fromDegrees(0.0));

    for (int i = 0; i <= m_trajectory.length - 1; ++i) {
      double[] trajectoryPoint = m_trajectory[i];
      double dt = trajectoryPoint[0];

      var desiredPose = new Pose2d(trajectoryPoint[1], trajectoryPoint[2],
          new Rotation2d(trajectoryPoint[3]));
      var desiredVelocity = trajectoryPoint[4];

      double desiredAngularVelocity;
      if (i == m_trajectory.length - 1) {
        desiredAngularVelocity = 0.0;
      } else {
        desiredAngularVelocity = boundRadians((m_trajectory[i + 1][3] - trajectoryPoint[3]) / dt);
      }

      var output = controller.calculate(desiredPose, desiredVelocity, desiredAngularVelocity,
          robotPose);
      robotPose = robotPose.exp(new Twist2d(output.linearVelocity * dt, 0,
          output.angularVelocity * dt));
    }

    Pose2d finalRobotPose = robotPose;
    double expectedX = m_trajectory[m_trajectory.length - 1][1];
    double expectedY = m_trajectory[m_trajectory.length - 1][2];
    double expectedTheta = m_trajectory[m_trajectory.length - 1][3];
    assertAll(
        () -> assertEquals(expectedX, finalRobotPose.getTranslation().getX(), kTolerance),
        () -> assertEquals(expectedY, finalRobotPose.getTranslation().getY(), kTolerance),
        () -> assertEquals(0.0,
            boundRadians(expectedTheta - finalRobotPose.getRotation().getRadians()),
            kAngularTolerance)
    );
  }

  private static double boundRadians(double value) {
    while (value > Math.PI) {
      value -= Math.PI * 2;
    }
    while (value <= -Math.PI) {
      value += Math.PI * 2;
    }
    return value;
  }
}
