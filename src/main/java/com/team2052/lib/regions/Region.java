package com.team2052.lib.regions;

import edu.wpi.first.math.geometry.Translation2d;

public abstract interface Region {

  public abstract boolean isPointInRegion(Translation2d point);

  /**
   * Returns true if the point is outside the region.
   *
   * @param point The point to check.
   * @return True if the point is outside the region.
   */
  public default boolean isPointOutsideRegion(Translation2d point) {
    return !isPointInRegion(point);
  }
}
