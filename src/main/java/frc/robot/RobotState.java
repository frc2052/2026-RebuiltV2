package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  @Setter @Getter private SwerveDriveState drivetrainState = new SwerveDriveState();
  @Setter @Getter private Pose2d autoStartPose;
  @Setter @Getter private Pose2d chassisVisionFieldPose = new Pose2d();
  @Setter @Getter private boolean firstShiftOverride = false;

  private static RobotState INSTANCE;

  public static RobotState getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotState();
    }
    return INSTANCE;
  }

  private RobotState() {}

  public Pose2d getFieldToRobot() {
    if (drivetrainState.Pose != null) {
      return drivetrainState.Pose;
    }

    return MathHelpers.POSE_2D_ZERO;
  }

  /*
   *  Robot Centric
   */
  public ChassisSpeeds getChassisSpeeds() {
    return drivetrainState.Speeds;
  }

  public double getRotationalSpeeds() {
    return drivetrainState.Speeds.omegaRadiansPerSecond;
  }

  public void toggleFirstShiftOverride() {
    firstShiftOverride = !firstShiftOverride;
  }

  public void output() {}
}
