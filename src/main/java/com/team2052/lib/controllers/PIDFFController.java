package com.team2052.lib.controllers;

import com.team2052.lib.util.FeedForwardConfiguration;
import com.team2052.lib.util.PIDConfiguration;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class PIDFFController {
  private ProfiledPIDController pidController;
  private SimpleMotorFeedforward feedforward;

  public PIDFFController(double p, double i, double d, double maxVelo, double maxAccel) {
    this(p, i, d, 0, 0, 0, maxVelo, maxAccel);
  }

  public PIDFFController(
      double p, double i, double d, double s, double v, double a, double maxVelo, double maxAccel) {
    pidController = new ProfiledPIDController(p, i, d, new Constraints(maxVelo, maxAccel));
    feedforward = new SimpleMotorFeedforward(s, v, a);
  }

  public PIDFFController(PIDConfiguration config) {
    this(config.kP, config.kI, config.kD, 0, 0, 0, config.maxVel, config.maxAcc);
  }

  public PIDFFController(PIDConfiguration pidConfig, FeedForwardConfiguration ffConfig) {
    this(
        pidConfig.kP,
        pidConfig.kI,
        pidConfig.kD,
        ffConfig.kS,
        ffConfig.kV,
        ffConfig.kA,
        pidConfig.maxVel,
        pidConfig.maxAcc);
  }

  public void enableContinuousInput(double min, double max) {
    pidController.enableContinuousInput(min, max);
  }

  public void setTolerance(double posTol, double velTol) {
    pidController.setTolerance(posTol, velTol);
  }

  public void setSetpoint(double s) {
    pidController.setGoal(s);
  }

  public double calculate(double measurement) {
    return pidController.calculate(measurement)
        + feedforward.calculate(pidController.calculate(measurement));
  }

  public double calculate(double measurement, double setpoint) {
    return pidController.calculate(measurement)
        + feedforward.calculate(pidController.calculate(measurement, setpoint));
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
