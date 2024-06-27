// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

@SuppressWarnings({"MemberName", "MissingJavadocMethod"})
public class Robot extends TimedRobot {
  public static final double dt = 0.01;
  public static final double offset = 0.005;

  public static final double Ks = 0.1;
  public static final double Kv = 3.0;
  public static final double Ka = 0.5;

  public static final double Kp = 0.1;
  public static final double Kd = 0.1;

  public Encoder encoder = new Encoder(0, 1);
  public PWMSparkMax motor = new PWMSparkMax(0);

  public ExponentialProfile profile =
      new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10.0, Kv, Ka));

  public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Ks, Kv, Ka);
  public PIDController feedback = new PIDController(Kp, 0.0, Kd, dt);

  public ExponentialProfile.State currentSetpoint = new ExponentialProfile.State();
  public ExponentialProfile.State goal = new ExponentialProfile.State();

  public Robot() {
    addPeriodic(this::controller, dt, offset);
  }

  public void setGoal(double goal) {
    this.goal.position = goal;
  }

  public void controller() {
    if (isDisabled()) {
      return;
    }

    var nextSetpoint = profile.calculate(dt, currentSetpoint, goal);
    motor.setVoltage(
        feedforward.calculate(currentSetpoint.velocity, nextSetpoint.velocity, dt)
            + feedback.calculate(encoder.getDistance(), currentSetpoint.position));
    currentSetpoint = nextSetpoint;
  }
}
