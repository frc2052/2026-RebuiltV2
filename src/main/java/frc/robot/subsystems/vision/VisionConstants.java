package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {

  public static final double CHASSIS_XY_STDDEV_COEFFICIENT = 0.04;
  public static final double TURRET_XY_STDDEV_COEFFICENT = 0.06;
  public static final double THETA_STDDEV_COEFFICIENT = Double.MAX_VALUE;
  public static final double DEFAULT_XY_STDDEV = 0.25;
  public static final double DEFAULT_HEADING_STDDEV = 0.1;
  public static final Matrix<N3, N1> VISION_STDDEV =
      new Matrix<>(VecBuilder.fill(DEFAULT_XY_STDDEV, DEFAULT_XY_STDDEV, Double.MAX_VALUE));
  public static final double MAX_POSE_AMBIGUITY = 0.15;
  public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);

  public static class BackLimelightConstants {
    public static final String CAMERA_NAME = "limelight-chassis";
    public static final Distance X_OFFSET = Inches.of(-12.86588);
    public static final Distance Y_OFFSET = Inches.of(4.5);
    public static final Distance Z_OFFSET = Inches.of(12.527);
    public static final Angle THETA_X_OFFSET = Degrees.of(0);
    public static final Angle THETA_Y_OFFSET = Degrees.of(0);
    public static final Angle THETA_Z_OFFSET = Degrees.of(180);
    public static final double[] LIMELIGHT_POSE = {
      X_OFFSET.in(Meters),
      Y_OFFSET.in(Meters),
      Z_OFFSET.in(Meters),
      THETA_X_OFFSET.in(Degrees),
      THETA_Y_OFFSET.in(Degrees),
      THETA_Z_OFFSET.in(Degrees)
    };
    public static final int MODE = 0;
  }

  public static class LeftLimelightConstants {
    public static final String CAMERA_NAME = "limelight-chassis";
    public static final Distance X_OFFSET = Inches.of(-12.86588);
    public static final Distance Y_OFFSET = Inches.of(4.5);
    public static final Distance Z_OFFSET = Inches.of(12.527);
    public static final Angle THETA_X_OFFSET = Degrees.of(0);
    public static final Angle THETA_Y_OFFSET = Degrees.of(0);
    public static final Angle THETA_Z_OFFSET = Degrees.of(180);
    public static final double[] LIMELIGHT_POSE = {
      X_OFFSET.in(Meters),
      Y_OFFSET.in(Meters),
      Z_OFFSET.in(Meters),
      THETA_X_OFFSET.in(Degrees),
      THETA_Y_OFFSET.in(Degrees),
      THETA_Z_OFFSET.in(Degrees)
    };
    public static final int MODE = 0;
  }

  public static class RightLimelightConstants {
    public static final String CAMERA_NAME = "limelight-chassis";
    public static final Distance X_OFFSET = Inches.of(-12.86588);
    public static final Distance Y_OFFSET = Inches.of(4.5);
    public static final Distance Z_OFFSET = Inches.of(12.527);
    public static final Angle THETA_X_OFFSET = Degrees.of(0);
    public static final Angle THETA_Y_OFFSET = Degrees.of(0);
    public static final Angle THETA_Z_OFFSET = Degrees.of(180);
    public static final double[] LIMELIGHT_POSE = {
      X_OFFSET.in(Meters),
      Y_OFFSET.in(Meters),
      Z_OFFSET.in(Meters),
      THETA_X_OFFSET.in(Degrees),
      THETA_Y_OFFSET.in(Degrees),
      THETA_Z_OFFSET.in(Degrees)
    };
    public static final int MODE = 0;
  }
}
