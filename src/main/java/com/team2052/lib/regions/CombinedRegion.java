package com.team2052.lib.regions;

import edu.wpi.first.math.geometry.Translation2d;

public class CombinedRegion implements Region {
  private Region[] regions;

  public CombinedRegion(Region... regions) {
    this.regions = regions;
  }

  public Region[] getRegions() {
    return regions;
  }

  public void setRegions(Region[] regions) {
    this.regions = regions;
  }

  public void addRegion(Region region) {
    Region[] newRegions = new Region[regions.length + 1];
    System.arraycopy(regions, 0, newRegions, 0, regions.length);
    newRegions[regions.length] = region;
    regions = newRegions;
  }

  @Override
  public boolean isPointInRegion(Translation2d point) {
    for (Region region : regions) {
      if (region.isPointInRegion(point)) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void logRegion(String folder) {
    if (!folder.endsWith("/")) folder += "/";
    for (int i = 0; i < regions.length; i++) {
      regions[i].logRegion(folder + "Region_" + i);
    }
  }
}
