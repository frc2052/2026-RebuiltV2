package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.regions.Region;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superstructure.shotTables.FeedingShootingTable;
import frc.robot.subsystems.superstructure.shotTables.HubShootingTable;
import frc.robot.subsystems.superstructure.shotTables.ShootingTableBase;
import frc.robot.subsystems.superstructure.timeTables.FeedingTimeTable;
import frc.robot.subsystems.superstructure.timeTables.HubTimeTable;
import frc.robot.subsystems.superstructure.timeTables.TimeTableBase;
import frc.robot.util.FieldConstants;
import frc.robot.util.MatchState;
import frc.robot.util.ShootingCalculator;
import frc.robot.util.ShotProfile;
import frc.robot.util.ShotProfile.TargetParameters;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

  private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private HoodSubsystem hood = HoodSubsystem.getInstance();

  @Getter @Setter private SuperstructureState currentState = SuperstructureState.NONE;
  @Getter private FieldRegion currentFieldRegion = FieldRegion.ALLIANCE_ZONE;

  @Getter @Setter private TargetType overrideTarget = TargetType.HUB;

  @Getter private ShotProfile lastCalculatedProfile;

  @Getter @Setter
  private Pair<AngularVelocity, Angle> manualShootingParameters =
      new Pair<>(RotationsPerSecond.of(0), Degrees.of(0));

  @Getter @Setter private boolean hasCalculatedShotProfileThisPeriod = false;

  @Getter @Setter private boolean isShootOnTheMove = SuperstructureConstants.DEFAULT_IS_SOTM;

  private static Superstructure INSTANCE;

  /**
   * Get the singleton instance of the Superstructure subsystem.
   *
   * @return the singleton instance of the Superstructure subsystem.
   */
  public static Superstructure getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Superstructure();
    }
    return INSTANCE;
  }

  private Superstructure() {}

  /** Pushes the current state and shooting parameters to the hood and shooter subsystems. */
  public void pushToSubsystems() {
    switch (currentState) {
      case NONE:
        hood.setToAngle(HoodConstants.HOOD_MIN_ANGLE);
        shooter.setGoalVelocity(RotationsPerSecond.of(0));
        break;
      case SHOOTING:
        hood.setToAngle(lastCalculatedProfile.aimingParameters.hoodAngle);
        shooter.setGoalVelocity(lastCalculatedProfile.aimingParameters.shooterVelocity);
        break;
      case OVERRIDE_SHOOTING:
        hood.setToAngle(lastCalculatedProfile.aimingParameters.hoodAngle);
        shooter.setGoalVelocity(lastCalculatedProfile.aimingParameters.shooterVelocity);
        break;
      case MANUAL:
        hood.setToAngle(manualShootingParameters.getSecond());
        shooter.setGoalVelocity(manualShootingParameters.getFirst());
        break;
      case TRENCH:
        hood.setToAngle(HoodConstants.HOOD_MIN_ANGLE);
        shooter.setGoalVelocity(ShooterConstants.IDLE_VELOCITY);
        break;
      default:
        // do nothing
        break;
    }
  }

  public Command setStateCommand(SuperstructureState newState) {
    return new InstantCommand(() -> setCurrentState(newState));
  }

  public void overrideTarget(TargetType newTarget) {
    overrideTarget = newTarget;
    setCurrentState(SuperstructureState.OVERRIDE_SHOOTING);
    recalculateShotProfile(newTarget);
  }

  public Command overrideTargetCommand(TargetType newTarget) {
    return new InstantCommand(() -> overrideTarget(newTarget));
  }

  /**
   * Set the manual shooting parameters for the shooter and hood.
   *
   * @param shooterVelocity the velocity to set the shooter to when in manual shooting mode.
   * @param hoodAngle the angle to set the hood to when in manual shooting mode.
   */
  public void setManualShootingParameters(AngularVelocity shooterVelocity, Angle hoodAngle) {
    manualShootingParameters = new Pair<>(shooterVelocity, hoodAngle);
  }

  /**
   * Set the manual shooting parameters for the shooter and hood based on a distance to target using
   * the hub shooting table.
   *
   * @param distanceToTarget the distance to the target to get the shooting parameters for.
   */
  public void setManualShootingParametersForHub(Distance distanceToTarget) {
    Pair<AngularVelocity, Angle> parameters =
        HubShootingTable.getInstance().getShootingParameters(distanceToTarget);
    setManualShootingParameters(parameters.getFirst(), parameters.getSecond());
  }

  @Override
  public void periodic() {
    determineFieldRegion();

    if (currentState == SuperstructureState.OVERRIDE_SHOOTING) {
      calculateShotProfile(overrideTarget);
    } else {
      calculateShotProfile();
    }
  }

  /**
   * Calculate the shot profile for the current period.
   *
   * @param targetType the target type to calculate the shot profile for.
   */
  public void calculateShotProfile(TargetType targetType) {
    if (hasCalculatedShotProfileThisPeriod) {
      return; // Prevent recalculating multiple times in the same period
    }

    lastCalculatedProfile =
        ShootingCalculator.calculateShotProfile(hasCalculatedShotProfileThisPeriod, targetType);
    hasCalculatedShotProfileThisPeriod = true;
  }

  /**
   * Calculate the shot profile for the current period. Uses the current field region to determine
   * which target use used.
   */
  public void calculateShotProfile() {
    calculateShotProfile(currentFieldRegion.getAssociatedTargetType());
  }

  /**
   * Call this method to force a recalculation of the shot profile for the current period, even if
   * it has already been calculated once.
   *
   * @param targetType the target type to recalculate the shot profile for.
   */
  public void recalculateShotProfile(TargetType targetType) {
    hasCalculatedShotProfileThisPeriod = false;
    calculateShotProfile(targetType);
  }

  /**
   * Call this method to force a recalculation of the shot profile for the current period, even if
   * it has already been calculated once.
   */
  public void recalculateShotProfile() {
    hasCalculatedShotProfileThisPeriod = false;
    calculateShotProfile();
  }

  private void determineFieldRegion() {
    Region allianceZone;
    Region depotSide;
    Region outpostSide;

    if (MatchState.isRedAlliance()) {
      allianceZone = FieldConstants.FieldRegions.RED_ALLIANCE_ZONE;
      depotSide = FieldConstants.FieldRegions.RED_DEPOT_SIDE;
      outpostSide = FieldConstants.FieldRegions.RED_OUTPOST_SIDE;
    } else {
      allianceZone = FieldConstants.FieldRegions.BLUE_ALLIANCE_ZONE;
      depotSide = FieldConstants.FieldRegions.BLUE_DEPOT_SIDE;
      outpostSide = FieldConstants.FieldRegions.BLUE_OUTPOST_SIDE;
    }

    Pose2d robotPose = RobotState.getInstance().getFieldToRobot();

    if (allianceZone.isPointInRegion(robotPose.getTranslation())) {
      currentFieldRegion = FieldRegion.ALLIANCE_ZONE;
    } else if (depotSide.isPointInRegion(robotPose.getTranslation())) {
      currentFieldRegion = FieldRegion.DEPOT_SIDE;
    } else if (outpostSide.isPointInRegion(robotPose.getTranslation())) {
      currentFieldRegion = FieldRegion.OUTPOST_SIDE;
    } else {
      currentFieldRegion = FieldRegion.NONE; // Not in any defined region
    }
  }

  public enum TargetType {
    /** The target type for the hub. */
    HUB(
        FieldConstants.FieldLocations.RED_ALLIANCE_HUB_LOCATION,
        FieldConstants.FieldLocations.BLUE_ALLIANCE_HUB_LOCATION,
        HubShootingTable.getInstance(),
        HubTimeTable.getInstance(),
        false),
    /**
     * The target type for feeding on the depot side of the field, flips halves depending on
     * alliance.
     */
    DEPOT_FEEDING(
        FieldConstants.FieldLocations.RED_ALLIANCE_DEPOT_SIDE_FEEDING_AIMING_POINT,
        FieldConstants.FieldLocations.BLUE_ALLIANCE_DEPOT_SIDE_FEEDING_AIMING_POINT,
        FeedingShootingTable.getInstance(),
        FeedingTimeTable.getInstance(),
        true),
    /**
     * The target type for feeding on the outpost side of the field, flips halves depending on
     * alliance.
     */
    OUTPOST_FEEDING(
        FieldConstants.FieldLocations.RED_ALLIANCE_OUTPOST_SIDE_FEEDING_AIMING_POINT,
        FieldConstants.FieldLocations.RED_ALLIANCE_OUTPOST_SIDE_FEEDING_AIMING_POINT,
        FeedingShootingTable.getInstance(),
        FeedingTimeTable.getInstance(),
        true);

    @Getter private final Translation2d redTargetLocation;
    @Getter private final Translation2d blueTargetLocation;
    @Getter private final ShootingTableBase shootingTable;
    @Getter private final TimeTableBase timeTable;
    @Getter private final boolean isFeeding;

    /** Get the target location for the current alliance. */
    public Translation2d getTargetLocation() {
      return MatchState.isRedAlliance() ? redTargetLocation : blueTargetLocation;
    }

    public TargetParameters toTargetParameters() {
      return new TargetParameters(getTargetLocation(), this);
    }

    private TargetType(
        Translation2d redTargetLocation,
        Translation2d blueTargetLocation,
        ShootingTableBase shootingTable,
        TimeTableBase timeTable,
        boolean isFeeding) {
      this.redTargetLocation = redTargetLocation;
      this.blueTargetLocation = blueTargetLocation;
      this.shootingTable = shootingTable;
      this.timeTable = timeTable;
      this.isFeeding = isFeeding;
    }
  }

  public enum SuperstructureState {
    TRENCH,
    MANUAL,
    SHOOTING,
    OVERRIDE_SHOOTING,
    NONE;
  }

  public enum FieldRegion {
    ALLIANCE_ZONE(
        FieldConstants.FieldRegions.RED_ALLIANCE_ZONE,
        FieldConstants.FieldRegions.BLUE_ALLIANCE_ZONE,
        TargetType.HUB),
    DEPOT_SIDE(
        FieldConstants.FieldRegions.RED_DEPOT_SIDE,
        FieldConstants.FieldRegions.BLUE_DEPOT_SIDE,
        TargetType.DEPOT_FEEDING),
    OUTPOST_SIDE(
        FieldConstants.FieldRegions.RED_OUTPOST_SIDE,
        FieldConstants.FieldRegions.BLUE_OUTPOST_SIDE,
        TargetType.OUTPOST_FEEDING),
    NONE(null, null, null);

    @Getter private final Region redRegion;
    @Getter private final Region blueRegion;
    @Getter private final TargetType associatedTargetType;

    private FieldRegion(Region redRegion, Region blueRegion, TargetType associatedTargetType) {
      this.redRegion = redRegion;
      this.blueRegion = blueRegion;
      this.associatedTargetType = associatedTargetType;
    }
  }
}
