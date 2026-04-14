package frc.robot.util;

import com.team2052.lib.geometry.Vector2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;

public class ShotProfile {

  public final AimingParameters aimingParameters;
  public final TargetParameters targetParameters;
  public final ShotInformation shotInformation;

  public ShotProfile(
      AimingParameters aimingParameters,
      TargetParameters targetParameters,
      ShotInformation shotInformation) {
    this.aimingParameters = aimingParameters;
    this.targetParameters = targetParameters;
    this.shotInformation = shotInformation;
  }

  public static class AimingParameters {
    public final AngularVelocity shooterVelocity;
    public final Angle hoodAngle;
    public final Rotation2d robotRotation;

    public AimingParameters(
        AngularVelocity shooterVelocity, Angle hoodAngle, Rotation2d robotRotation) {
      this.shooterVelocity = shooterVelocity;
      this.hoodAngle = hoodAngle;
      this.robotRotation = robotRotation;
    }
  }

  public static class TargetParameters {
    public final Translation2d targetPosition;
    public final TargetType targetType;

    public TargetParameters(Translation2d targetPosition, TargetType targetType) {
      this.targetPosition = targetPosition;
      this.targetType = targetType;
    }
  }

  public static class ShotInformation {
    public final boolean isShootOnTheMove;
    public final Vector2d robotVelocity;
    public final Time shotTime;

    public ShotInformation(boolean isShootOnTheMove, Vector2d robotVelocity, Time shotTime) {
      this.isShootOnTheMove = isShootOnTheMove;
      this.robotVelocity = robotVelocity;
      this.shotTime = shotTime;
    }
  }
}
