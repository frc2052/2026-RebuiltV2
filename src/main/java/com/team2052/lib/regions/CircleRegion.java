package com.team2052.lib.regions;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class CircleRegion implements Region {
  private final Translation2d center;
  private final Distance radius;

  public CircleRegion(Translation2d center, Distance radius) {
    this.center = center;
    this.radius = radius;
  }

  @Override
  public boolean isPointInRegion(Translation2d point) {
    return center.getDistance(point) <= radius.in(Meters);
  }

}
