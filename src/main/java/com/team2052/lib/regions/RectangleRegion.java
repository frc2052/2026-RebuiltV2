package com.team2052.lib.regions;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RectangleRegion implements Region {
  Rectangle2d region;

  public RectangleRegion(Rectangle2d region) {
    this.region = region;
  }

  public RectangleRegion(Translation2d cornerA, Translation2d cornerB) {
    this.region = new Rectangle2d(cornerA, cornerB);
  }

  @Override
  public boolean isPointInRegion(Translation2d point) {
    return region.contains(point);
  }

  @Override
  public void logRegion(String folder) {
    if (!folder.endsWith("/")) folder += "/";
    // Logger.recordOutput(folder + "RectangleRegion/Rectangle", region);
  }
}
