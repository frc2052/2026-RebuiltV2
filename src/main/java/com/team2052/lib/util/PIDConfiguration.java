package com.team2052.lib.util;

public class PIDConfiguration {

  public final double kP;
  public final double kI;
  public final double kD;

  public final double maxAcc;
  public final double maxVel;

  public PIDConfiguration(double kP, double kI, double kD, double maxVel, double maxAcc) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.maxVel = maxVel;
    this.maxAcc = maxAcc;
  }
}
