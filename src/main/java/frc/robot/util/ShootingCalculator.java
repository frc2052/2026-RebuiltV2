package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.team2052.lib.geometry.Vector2d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.util.ShotProfile.*;

public class ShootingCalculator {

  public static ShotProfile calculateShotProfile(boolean isSOTM, TargetType targetType) {
    TargetParameters targetParameters = getTargetParameters(targetType);
    Vector2d robotVelocity = getFieldRelativeRobotVelocity();

    return isSOTM
        ? calculateSOTMShotProfile(targetParameters, robotVelocity, isSOTM)
        : calculateStaticShotProfile(targetParameters, robotVelocity, isSOTM);
  }

  private static TargetParameters getTargetParameters(TargetType targetType) {
    return new TargetParameters(targetType.getTargetLocation(), targetType);
  }

  private static Vector2d getFieldRelativeRobotVelocity() {
    ChassisSpeeds speeds = RobotState.getInstance().getChassisSpeeds();
    Vector2d robotRelativeVelocity =
        new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return robotRelativeVelocity.rotated(RobotState.getInstance().getFieldToRobot().getRotation());
  }

  private static ShotProfile calculateStaticShotProfile(
      TargetParameters targetParameters, Vector2d robotVelocity, boolean isSOTM) {
    Pose2d robotPose = RobotState.getInstance().getFieldToRobot();
    Distance distanceToTarget =
        Meters.of(robotPose.getTranslation().getDistance(targetParameters.targetPosition));

    if (targetParameters.targetType.isFeeding()) {
      targetParameters =
          new TargetParameters(
              new Translation2d(
                  targetParameters.targetPosition.getX(), robotPose.getTranslation().getY()),
              targetParameters.targetType);
    }

    Pair<AngularVelocity, Angle> shootingParameters =
        targetParameters.targetType.getShootingTable().getShootingParameters(distanceToTarget);

    Rotation2d rotationToTarget =
        targetParameters
            .targetPosition
            .minus(robotPose.getTranslation())
            .getAngle()
            .plus(Rotation2d.k180deg);
    AimingParameters aimingParameters =
        new AimingParameters(
            shootingParameters.getFirst(), shootingParameters.getSecond(), rotationToTarget);
    ShotInformation shotInformation =
        new ShotInformation(
            isSOTM,
            robotVelocity,
            targetParameters.targetType.getTimeTable().getTime(distanceToTarget));

    return new ShotProfile(aimingParameters, targetParameters, shotInformation);
  }

  private static ShotProfile calculateSOTMShotProfile(
      TargetParameters targetParameters, Vector2d robotVelocity, boolean isSOTM) {
    Pose2d robotPose = RobotState.getInstance().getFieldToRobot();

    if (targetParameters.targetType.isFeeding()) {
      targetParameters =
          new TargetParameters(
              new Translation2d(
                  targetParameters.targetPosition.getX(), robotPose.getTranslation().getY()),
              targetParameters.targetType);
    }

    Vector2d sotmOffsetVector = calculateSOTMOffsetVector(targetParameters, robotVelocity);
    Pose2d adjustedRobotPose =
        new Pose2d(
            robotPose.getTranslation().plus(sotmOffsetVector.getVectorAsTranslation()),
            robotPose.getRotation());

    Distance adjustedDistanceToTarget =
        Meters.of(adjustedRobotPose.getTranslation().getDistance(targetParameters.targetPosition));
    Pair<AngularVelocity, Angle> shootingParameters =
        targetParameters
            .targetType
            .getShootingTable()
            .getShootingParameters(adjustedDistanceToTarget);

    Rotation2d rotationToTarget =
        targetParameters
            .targetPosition
            .minus(adjustedRobotPose.getTranslation())
            .getAngle()
            .plus(Rotation2d.k180deg);
    AimingParameters aimingParameters =
        new AimingParameters(
            shootingParameters.getFirst(), shootingParameters.getSecond(), rotationToTarget);
    ShotInformation shotInformation =
        new ShotInformation(
            isSOTM,
            robotVelocity,
            targetParameters.targetType.getTimeTable().getTime(adjustedDistanceToTarget));

    return new ShotProfile(aimingParameters, targetParameters, shotInformation);
  }

  private static Vector2d calculateSOTMOffsetVector(
      TargetParameters targetParameters, Vector2d robotVelocity) {
    Pose2d robotPose = RobotState.getInstance().getFieldToRobot();

    Distance initialDistance =
        Meters.of(robotPose.getTranslation().getDistance(targetParameters.targetPosition));
    Time initialTimeGuess = targetParameters.targetType.getTimeTable().getTime(initialDistance);

    // Iteratively refine the guess and offset vector
    Vector2d offsetVector = robotVelocity.scaled(initialTimeGuess.in(Seconds));
    Time timeToTarget = initialTimeGuess;

    for (int i = 0;
        i < SuperstructureConstants.SOTM_ITERATIONS;
        i++) { // Limit iterations to prevent infinite loop
      Pose2d adjustedPose =
          new Pose2d(
              robotPose.getTranslation().plus(offsetVector.getVectorAsTranslation()),
              robotPose.getRotation());
      Distance adjustedDistance =
          Meters.of(adjustedPose.getTranslation().getDistance(targetParameters.targetPosition));

      Time newTimeToTarget = targetParameters.targetType.getTimeTable().getTime(adjustedDistance);
      Time timeDifference = newTimeToTarget.minus(timeToTarget);

      if (timeDifference.in(Seconds)
          < SuperstructureConstants.SOTM_CONVERGENCE_THRESHOLD_METERS) { // Convergence threshold
        offsetVector = robotVelocity.scaled(newTimeToTarget.in(Seconds));
        break;
      }

      double dampeningFactor =
          SuperstructureConstants
              .SOTM_DAMPENING_FACTOR; // Damping factor to prevent overshooting and oscillation
      timeToTarget = timeToTarget.plus(timeDifference.times(dampeningFactor));
      offsetVector = robotVelocity.scaled(timeToTarget.in(Seconds));
    }

    return offsetVector;
  }
}
