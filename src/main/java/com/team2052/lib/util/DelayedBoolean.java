package com.team2052.lib.util;

/** An iterative boolean latch that delays the transition from false to true. */
public class DelayedBoolean {
  private boolean lastVal;
  private double transitionTimestamp;
  private final double mDelay;

  public DelayedBoolean(double timestamp, double delay) {
    transitionTimestamp = timestamp;
    lastVal = false;
    mDelay = delay;
  }

  public boolean update(double timestamp, boolean value) {
    boolean result = false;

    if (value && !lastVal) {
      transitionTimestamp = timestamp;
    }

    // If we are still true and we have transitioned.
    if (value && (timestamp - transitionTimestamp > mDelay)) {
      result = true;
    }

    lastVal = value;
    return result;
  }
}
