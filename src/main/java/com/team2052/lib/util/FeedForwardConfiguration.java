package com.team2052.lib.util;

public class FeedForwardConfiguration {

  public final double kS;
  public final double kV;
  public final double kA;

  public FeedForwardConfiguration(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
