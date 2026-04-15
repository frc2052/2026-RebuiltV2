package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.vision.limelight.LimelightHelpers;
import com.team2052.lib.vision.limelight.LimelightHelpers.PoseEstimate;
import com.team2052.lib.vision.limelight.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.util.FieldConstants;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;

public class VisionSubsystem extends SubsystemBase {
  private StatusSignal<Angle> yawSignal = DrivetrainSubsystem.getInstance().getPigeon2().getYaw();
  private StatusSignal<Angle> pitchSignal =
      DrivetrainSubsystem.getInstance().getPigeon2().getPitch();
  private StatusSignal<Angle> rollSignal = DrivetrainSubsystem.getInstance().getPigeon2().getRoll();

  private PoseEstimate previousChassisEstimate;

  private final RobotState robotState = RobotState.getInstance();

  private static VisionSubsystem INSTANCE;

  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem();
    }

    return INSTANCE;
  }

  private VisionSubsystem() {
    configureLimelights();
    yawSignal.setUpdateFrequency(100);
    pitchSignal.setUpdateFrequency(20);
    rollSignal.setUpdateFrequency(20);
  }

  private void configureLimelights() {
    LimelightCamera.CHASSIS
        .getTable()
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(BackLimelightConstants.LIMELIGHT_POSE);
    LimelightCamera.LEFT
        .getTable()
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(LeftLimelightConstants.LIMELIGHT_POSE);
    LimelightCamera.RIGHT
        .getTable()
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(RightLimelightConstants.LIMELIGHT_POSE);

    // set double to 1 for enable, 0 to disable
    LimelightCamera.CHASSIS.getTable().getEntry("rewind_enable_set").setDouble(0);
    LimelightCamera.LEFT.getTable().getEntry("rewind_enable_set").setDouble(0);
    LimelightCamera.RIGHT.getTable().getEntry("rewind_enable_set").setDouble(0);
  }

  public void visionPeriodic() {
    filter(readMT1(LimelightCamera.CHASSIS, previousChassisEstimate))
        .ifPresent(
            e -> {
              RobotState.getInstance().setChassisVisionFieldPose(e.pose);
              DrivetrainSubsystem.getInstance()
                  .addVisionMeasurement(
                      e.pose,
                      Utils.fpgaToCurrentTime(e.timestampSeconds),
                      calculateStandardDeviation(e));
            });

    filter(readMT1(LimelightCamera.LEFT, previousChassisEstimate))
        .ifPresent(
            e -> {
              RobotState.getInstance().setChassisVisionFieldPose(e.pose);
              DrivetrainSubsystem.getInstance()
                  .addVisionMeasurement(
                      e.pose,
                      Utils.fpgaToCurrentTime(e.timestampSeconds),
                      calculateStandardDeviation(e));
            });

    filter(readMT1(LimelightCamera.RIGHT, previousChassisEstimate))
        .ifPresent(
            e -> {
              RobotState.getInstance().setChassisVisionFieldPose(e.pose);
              DrivetrainSubsystem.getInstance()
                  .addVisionMeasurement(
                      e.pose,
                      Utils.fpgaToCurrentTime(e.timestampSeconds),
                      calculateStandardDeviation(e));
            });

    pushYaw(LimelightCamera.CHASSIS);
    pushYaw(LimelightCamera.LEFT);
    pushYaw(LimelightCamera.RIGHT);

    NetworkTableInstance.getDefault().flush();
  }

  public void pushYaw(LimelightCamera camera) {
    Angle yaw = yawSignal.getValue();
    Angle pitch = pitchSignal.getValue();
    Angle roll = rollSignal.getValue();

    double[] entries = new double[6];
    entries[0] = yaw.in(Degrees); // yaw
    entries[1] = 0; // yaw rate
    entries[2] = pitch.in(Degrees); // pitch
    entries[3] = 0; // pitch rate
    entries[4] = roll.in(Degrees); // roll
    entries[5] = 0; // roll rate

    camera.getTable().getEntry("robot_orientation_set").setDoubleArray(entries);
  }

  private Optional<PoseEstimate> readMT1(LimelightCamera type, PoseEstimate previousEstimate) {
    String name = type.getCameraName();
    if (LimelightHelpers.getTV(name)) {
      try {
        double oldTimestamp =
            previousEstimate != null ? previousEstimate.timestampSeconds : Double.MAX_VALUE;
        PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (newEstimate.timestampSeconds == oldTimestamp) {
          return Optional.empty(); // no new data
        }

        previousEstimate = newEstimate;
        return Optional.of(newEstimate);
      } catch (Exception e) {
        System.err.println("Error retrieving " + name + " data: " + e.getMessage());
      }
    }

    return Optional.empty();
  }

  private Optional<PoseEstimate> readMT2(LimelightCamera type, PoseEstimate previousEstimate) {
    String name = type.getCameraName();

    if (LimelightHelpers.getTV(name)) {
      try {
        double oldTimestamp =
            previousEstimate != null ? previousEstimate.timestampSeconds : Double.MAX_VALUE;
        PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (newEstimate.timestampSeconds == oldTimestamp) {
          return Optional.empty(); // no new data
        }

        previousEstimate = newEstimate;
        return Optional.of(newEstimate);
      } catch (Exception e) {
        System.err.println("Error retrieving " + name + "  data: " + e.getMessage());
      }
    }
    return Optional.empty();
  }

  // Filters out estimates when the robot is moving too fast or the estimate is outside the field
  private Optional<PoseEstimate> filter(Optional<PoseEstimate> estimate) {
    return estimate.filter(
        e -> {
          boolean speed = MathHelpers.chassisSpeedsNorm(robotState.getChassisSpeeds()) > 3;
          boolean field =
              e.pose.getTranslation().getX() < -VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
                  || e.pose.getTranslation().getX()
                      > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
                  || e.pose.getTranslation().getY()
                      < -VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
                  || e.pose.getTranslation().getY()
                      > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN.in(Meters);
          return !(speed || field);
        });
  }

  private Vector<N3> calculateStandardDeviation(PoseEstimate estimate) {

    OptionalDouble optStdDev =
        Arrays.stream(estimate.rawFiducials).mapToDouble(fiducial -> fiducial.distToCamera).min();
    double stdDev = optStdDev.isPresent() ? optStdDev.getAsDouble() : Double.MAX_VALUE;
    double headingStdDev = Double.MAX_VALUE;

    double closestTagDist = Double.MAX_VALUE;
    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.distToCamera < closestTagDist) {
        closestTagDist = fiducial.distToCamera;
      }
    }
    if (closestTagDist < 1) closestTagDist = 1;

    stdDev =
        VisionConstants.CHASSIS_XY_STDDEV_COEFFICIENT
                * Math.pow(closestTagDist, 2)
                / estimate.tagCount
            + VisionConstants.DEFAULT_XY_STDDEV;

    return VecBuilder.fill(stdDev, stdDev, headingStdDev);
  }

  public enum LimelightCamera {
    CHASSIS(BackLimelightConstants.CAMERA_NAME),
    LEFT(LeftLimelightConstants.CAMERA_NAME),
    RIGHT(RightLimelightConstants.CAMERA_NAME);

    private final String cameraName;
    private final NetworkTable table;

    LimelightCamera(String cameraName) {
      this.cameraName = cameraName;
      this.table = NetworkTableInstance.getDefault().getTable(cameraName);
    }

    public String getCameraName() {
      return cameraName;
    }

    public NetworkTable getTable() {
      return table;
    }
  }
}
