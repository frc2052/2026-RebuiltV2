package com.team2052.lib.helpers;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

public class MathHelpers {
  public static final Pose2d POSE_2D_ZERO = new Pose2d();

  public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
    return new Pose2d(TRANSLATION_2D_ZERO, rotation);
  }

  public static final Pose2d pose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, ROTATION_2D_ZERO);
  }

  public static final Rotation2d ROTATION_2D_ZERO = new Rotation2d();
  public static final Rotation2d ROTATION_2D_PI = Rotation2d.fromDegrees(180.0);

  public static final Translation2d TRANSLATION_2D_ZERO = new Translation2d();

  public static double deadband(double value, double deadband) {
    deadband = Math.abs(deadband);
    if (deadband == 1) {
      return 0;
    }
    double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
    return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
  }

  /**
   * Returns the norm of the translational components of a ChassisSpeeds object, which is just the
   * hypotenuse of the x and y velocities.
   */
  public static double chassisSpeedsNorm(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Returns the Transform3d with the smaller translation norm.
   *
   * @see MathHelpers.chassisSpeedsNorm(ChassisSpeeds) for how the translation norm is calculated.
   * @param t1
   * @param t2
   * @return smallest transform
   */
  public static Transform3d getSmallestTransform(Transform3d t1, Transform3d t2) {
    if (t1.getTranslation().getNorm() < t2.getTranslation().getNorm()) {
      return t1;
    }

    return t2;
  }

  /**
   * Checks if two doubles are equal within a given epsilon (difference).
   *
   * @param a
   * @param b
   * @param epsilon
   * @return
   */
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /** Clamps a value between a minimum and maximum value. */
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public static int modInverse(int value, int modulus) {
    int originalModulus = modulus;

    int previousCoefficient = 0;
    int currentCoefficient = 1;

    int previousRemainder = modulus;
    int currentRemainder = value;

    while (currentRemainder != 0) {
      int quotient = previousRemainder / currentRemainder;

      int nextRemainder = previousRemainder - quotient * currentRemainder;
      previousRemainder = currentRemainder;
      currentRemainder = nextRemainder;

      int nextCoefficient = previousCoefficient - quotient * currentCoefficient;
      previousCoefficient = currentCoefficient;
      currentCoefficient = nextCoefficient;
    }

    if (previousRemainder != 1) {
      throw new IllegalArgumentException("No modular inverse exists");
    }

    int inverse = previousCoefficient;
    if (inverse < 0) inverse += originalModulus;

    return inverse;
  }

  public static boolean angleEpsilonEquals(Angle angle1, Angle angle2, Angle epsilon) {
    // pi to -pi
    double angle1Rads = MathUtil.angleModulus(angle1.in(Radians));
    double angle2Rads = MathUtil.angleModulus(angle2.in(Radians));

    double difference0 = Math.abs(angle1Rads - angle2Rads);
    double difference1 = Math.abs(angle1Rads - angle2Rads + 2 * Math.PI);
    double difference2 = Math.abs(angle1Rads - angle2Rads - 2 * Math.PI);

    return (difference0 <= epsilon.in(Radians)
        || difference1 <= epsilon.in(Radians)
        || difference2 <= epsilon.in(Radians));
  }
}
