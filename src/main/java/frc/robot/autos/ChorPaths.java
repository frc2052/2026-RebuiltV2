package frc.robot.autos;

public enum ChorPaths {
  PRELOAD_ONLY_LEFT("PRELOAD_ONLY_LEFT"), 
  PRELOAD_ONLY_RIGHT("PRELOAD_ONLY_RIGHT"), 

  // left
  LTRENCH_LNEUTRAL1("LTRENCH_LNEUTRAL1"), 
  LNEUTRAL_LTRENCH("LNEUTRAL_LTRENCH"), 
  LTRENCH_HUB("LT_HUB"), 
  LNEUTRAL_LBUMP("LNEUTRAL_LBUMP"), 

  // right
  RTRENCH_RNEUTRAL1("RTRENCH_RNEUTRAL1"),
  RNEUTRAL_RTRENCH("RNEUTRAL_RTRENCH"), 
  RTRENCH_HUB("RTRENCH_HUB"), 
  RNEUTRAL_RBUMP("RNEUTRAL_RBUMP"),

  // center
  CENTER_BACKUP("CENTER_BACKUP");

  public String pathName;

  private ChorPaths(String pName) {
    pathName = pName;
  }

  public String getPathName() {
    return pathName;
  }
}
