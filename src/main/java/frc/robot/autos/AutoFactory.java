package frc.robot.autos;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.FiringCommand;
import frc.robot.commands.drive.AimingDriveCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakePosition;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.Set;

// spotless: off
public class AutoFactory {
  static final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  static final NetworkTable debugTable = networkTables.getTable("autos networktables tab");
  static final DoubleTopic waitTimeTopic = debugTable.getDoubleTopic("waitTime");
  static final DoublePublisher waitTimePublisher = waitTimeTopic.publish();
  static final DoubleSubscriber waitTimeSubscriber = waitTimeTopic.subscribe(0);

  BooleanTopic sprintTopic = debugTable.getBooleanTopic("SPRINT?");
  BooleanPublisher sprintPublisher = sprintTopic.publish();
  BooleanSubscriber sprintSubscriber = sprintTopic.subscribe(false);

  BooleanTopic returnTopic = debugTable.getBooleanTopic("RETURN? Y: TRENCH N: BUMP");
  BooleanPublisher returnPublisher = returnTopic.publish();
  BooleanSubscriber returnSubscriber = sprintTopic.subscribe(false);

  public AutoFactory() {

    sprintTopic.publish().accept(false);
    sprintPublisher.set(false);

    returnTopic.publish().accept(false);
    returnPublisher.set(false);

    waitTimeTopic.publish().accept(0);
    waitTimePublisher.set(0);
  }

  public enum SimpleScoringLocation {
    OUTPOST(39.72, 4.57),
    DEPOT(-106.435546875, 4.375);

    public double degrees;
    public double distanceFromHubMeters;

    private SimpleScoringLocation(double d, double dist) {
      degrees = d;
      distanceFromHubMeters = dist;
    }

    public double getDegrees() {
      return degrees;
    }

    public double getDistToHub() {
      return distanceFromHubMeters;
    }
  }

  // -------------------------------- ACTUAL AUTOS -------------------------------- //

  Pair<Pose2d, Command> noAuto() {
    return Pair.of(new Pose2d(), Commands.none());
  }

  Pair<Pose2d, Command> preloadOnlyLeft() {
    return Pair.of(
        getStartPose(ChorPaths.PRELOAD_ONLY_LEFT),
        Commands.sequence(
            followPathCommand(ChorPaths.PRELOAD_ONLY_LEFT),
            simpleScore(TargetType.HUB, 5),
            postScoringCleanup()));
  }

  Pair<Pose2d, Command> preloadOnlyRight() {
    return Pair.of(
        getStartPose(ChorPaths.PRELOAD_ONLY_RIGHT),
        Commands.sequence(
            followPathCommand(ChorPaths.PRELOAD_ONLY_RIGHT),
            simpleScore(TargetType.HUB, 5),
            postScoringCleanup()));
  }

  Pair<Pose2d, Command> preloadOnlyCenter() {
    return Pair.of(
        getStartPose(ChorPaths.CENTER_BACKUP),
        Commands.sequence(
            followPathCommand(ChorPaths.CENTER_BACKUP),
            simpleScore(TargetType.HUB, 5),
            postScoringCleanup()));
  }

  // LEFT AUTOS

  Pair<Pose2d, Command> leftSingleTrenchSweep() {
    return Pair.of(
        getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
        Commands.sequence(
            postScoringCleanup(),
            // Commands.waitSeconds(0.2), // wait b4 going under trench
            Commands.deadline(
                Commands.sequence(
                    followPathCommand(ChorPaths.LTRENCH_LNEUTRAL1), //
                    followPathCommand(ChorPaths.LNEUTRAL_LBUMP)),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 2), // reduced for PM6 auto
            postScoringCleanup()));
  }

  Pair<Pose2d, Command> leftDoubleSweep() {
    return Pair.of(
        getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
        Commands.sequence(
            leftSingleTrenchSweep().getSecond(),
            IntakePivotSubsystem.getInstance().setCommand(IntakePosition.OUT_POSITION),
            Commands.deadline(
                Commands.sequence(
                    followPathCommand(ChorPaths.LBUMP_HUB),
                    followPathCommand(ChorPaths.LNEUTRAL_LBUMP)),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 2.5)));
  }

  // RIGHT AUTOS
  Pair<Pose2d, Command> rightSingleTrenchSweep() {
    return Pair.of(
        getStartPose(ChorPaths.RTRENCH_RNEUTRAL1),
        Commands.sequence(
            postScoringCleanup(),
            Commands.deadline(
                Commands.sequence(
                    followPathCommand(ChorPaths.RTRENCH_RNEUTRAL1),
                    followPathCommand(ChorPaths.RNEUTRAL_RBUMP)),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 2), // reduced for PM6 auto
            postScoringCleanup()));
  }

  Pair<Pose2d, Command> rightDoubleSweep() {
    return Pair.of(
        getStartPose(ChorPaths.RTRENCH_RNEUTRAL1),
        Commands.sequence(
            rightSingleTrenchSweep().getSecond(),
            IntakePivotSubsystem.getInstance().setCommand(IntakePosition.OUT_POSITION),
            Commands.deadline(
                Commands.sequence(
                    followPathCommand(ChorPaths.RBUMP_HUB),
                    followPathCommand(ChorPaths.RNEUTRAL_RBUMP)),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 2.5)));
  }

  // full sweeps (left & right)

  Pair<Pose2d, Command> rightFullSweep() {
    return Pair.of(
        getStartPose(ChorPaths.RBUMP_SWEIP1),
        Commands.sequence(
            followPathCommand(ChorPaths.RBUMP_SWEIP1),
            Commands.waitSeconds(getWaitSeconds()),
            Commands.deadline(
                followPathCommand(ChorPaths.RFULL_SWEEP2),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            returnTrenchOrBump(returnSubscriber.get()),
            simpleScore(TargetType.HUB, 4)));
  }

  Pair<Pose2d, Command> RCSwipe() {
    return Pair.of(
        getStartPose(ChorPaths.RC),
        Commands.sequence(
            Commands.deadline(
                followPathCommand(ChorPaths.RC),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    intakePivot.setCommand(IntakePosition.OUT_POSITION),
                    IntakeRollerSubsystem.getInstance().runIntakeCommand())),
            Commands.deadline(
                followPathCommand(ChorPaths.RC_RETURN),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 3)));
  }

  Pair<Pose2d, Command> LCSWipe() {
    return Pair.of(
        getStartPose(ChorPaths.LC),
        Commands.sequence(
            Commands.deadline(
                followPathCommand(ChorPaths.LC),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    intakePivot.setCommand(IntakePosition.OUT_POSITION),
                    IntakeRollerSubsystem.getInstance().runIntakeCommand())),
            Commands.deadline(
                followPathCommand(ChorPaths.LC_RETURN),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            scoreWhileActuatingHalfway(TargetType.HUB, 3)));
  }

  //
  Pair<Pose2d, Command> rightDelayBumpSwipe() {
    return Pair.of(
        getStartPose(ChorPaths.RBUMP_SWEIP1),
        Commands.sequence(
            followPathCommand(ChorPaths.RBUMP_SWEIP1),
            Commands.waitSeconds(getWaitSeconds()),
            intakePivot.setCommand(IntakePosition.OUT_POSITION),
            Commands.deadline(
                followPathCommand(ChorPaths.MID_SWIPE),
                IntakeRollerSubsystem.getInstance().runAtVelocityCommand(null)),
            followPathCommand(ChorPaths.RNEUTRAL_RBUMP),
            simpleScore(TargetType.HUB, 3)));
  }

  Pair<Pose2d, Command> leftDelayBumpSwipe() {
    return Pair.of(
        getStartPose(ChorPaths.LBUMP_SWIPE1),
        Commands.sequence(
            followPathCommand(ChorPaths.LBUMP_SWIPE1),
            Commands.waitSeconds(getWaitSeconds()),
            intakePivot.setCommand(IntakePosition.OUT_POSITION),
            Commands.deadline(
                followPathCommand(ChorPaths.MID_SWIPE),
                IntakeRollerSubsystem.getInstance().runAtVelocityCommand(null)),
            followPathCommand(ChorPaths.LNEUTRAL_LBUMP),
            simpleScore(TargetType.HUB, 3)));
  }

  // Pair<Pose2d, Command> leftDelayTrenchSwipe() {
  //   return Pair.of(
  //       getStartPose(ChorPaths.LT_DELAY_SWIPE),
  //       Commands.sequence(
  //           // wait for other team to pickup, then enter NZ
  //           Commands.waitSeconds(3),
  //           Commands.deadline(
  //               Commands.sequence(
  //                   followPathCommand(ChorPaths.LT_DELAY_SWIPE),
  //                   followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
  //               IntakeRollerSubsystem.getInstance().runIntakeCommand()),
  //           // score pickup
  //           simpleScore(TargetType.HUB, 3)
  //           // outpost?
  //           ));
  // }

  // Pair<Pose2d, Command> rightDelayTrenchSwipe() {
  //   return Pair.of(
  //       getStartPose(ChorPaths.RT_DELAY_SWIPE),
  //       Commands.sequence(
  //           // wait for other team to pickup, thene enter NZ
  //           Commands.waitSeconds(3),
  //           Commands.deadline(
  //               Commands.sequence(
  //                   followPathCommand(ChorPaths.RT_DELAY_SWIPE),
  //                   followPathCommand(ChorPaths.RNEUTRAL_RTRENCH)),
  //               IntakeRollerSubsystem.getInstance().runIntakeCommand()),
  //           // score pickup
  //           simpleScore(TargetType.HUB, 3)));
  // }

  // -------------------------------- FACTORY METHODS -------------------------------- //
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  public final VisionSubsystem vision = VisionSubsystem.getInstance();
  public final FeederSubsystem feeder = FeederSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final StagerSubsystem stager = StagerSubsystem.getInstance();
  public final IntakeRollerSubsystem intake = IntakeRollerSubsystem.getInstance();
  public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
  public final Superstructure superstructure = Superstructure.getInstance();
  public final HoodSubsystem hood = HoodSubsystem.getInstance();

  // wait seconds
  public double getWaitSeconds() {
    return waitTimeSubscriber.get();
  }

  // other
  public static Pose2d getAllianceAdjustedPose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      // return new Pose2d(
      //     new Translation2d(16.54 - bluePose.getX(), 18.07 - bluePose.getY()),
      //     bluePose.getRotation().minus(Rotation2d.kPi));
      // return ChoreoAllianceFlipUtil.flip(bluePose);
      return FlippingUtil.flipFieldPose(bluePose);
    } else {
      return bluePose;
    }
  }

  public Command returnTrenchOrBump(boolean left) {
    return Commands.defer(
        () -> {
          if (returnSubscriber.get()) {
            if (left) {
              return followPathCommand(ChorPaths.LNEUTRAL_LTRENCH);
              // left trench return
            } else {
              return followPathCommand(ChorPaths.RTRENCH_RNEUTRAL1);
              // right trench return
            }
          } else {
            if (left) {
              return followPathCommand(ChorPaths.LNEUTRAL_LBUMP);
              // left bump return
            } else {
              return followPathCommand(ChorPaths.RNEUTRAL_RBUMP);
              // right bump return
            }
          }
        },
        Set.of());
  }

  public Command sprintOrScorePreload() {
    return Commands.defer(
        () -> {
          if (sprintSubscriber.get()) {
            return Commands.none();
          } else {
            return simpleScore(TargetType.HUB, 2);
          }
        },
        Set.of());
  }

  public Command waitSuppliedSeconds() {
    return Commands.defer(
        () -> {
          double wait = waitTimeSubscriber.get();
          System.out.println("==== AUTO WAIT SECONDS SET: " + wait + " seconds");
          return Commands.waitSeconds(wait);
        },
        Set.of());
  }

  public Command resetGyro() {
    return Commands.runOnce(() -> drivetrain.seedFieldCentric());
  }

  public Command setState(SuperstructureState state) {
    return superstructure.setStateCommand(state);
  }

  // intaking
  public Command intakeWithTime(double seconds) {
    return Commands.sequence(
        intakePivot.setCommand(IntakePosition.OUT_POSITION),
        intake.runIntakeCommand().withTimeout(seconds));
  }

  public Command indexAll() {
    return Commands.runOnce(() -> floor.scoreCommand());
  }

  // shooting

  public Command postScoringCleanup() {
    return Commands.sequence(
        Commands.runOnce(
            () -> Superstructure.getInstance().setCurrentState(SuperstructureState.TRENCH)),
        IntakePivotSubsystem.getInstance().setCommand(IntakePosition.OUT_POSITION));
  }

  public Command firingCommand() {
    return new FiringCommand();
  }

  public Command hubScoringState() {
    return Commands.runOnce(() -> superstructure.setStateCommand(SuperstructureState.SHOOTING));
  }

  public Command trenchScoringState() {
    return Commands.runOnce(() -> superstructure.setStateCommand(SuperstructureState.TRENCH));
  }

  public Command simpleScore(TargetType target, double scoreTime) {
    return Commands.deadline(
            // checks for shooter to be at goal velocity; fire for scoreTime
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.waitUntil(
                        () ->
                            (shooter.isAtGoalVelocity(RotationsPerSecond.of(1))
                                && MathHelpers.epsilonEquals(
                                    MathUtil.angleModulus(
                                        superstructure
                                                .getLastCalculatedProfile()
                                                .aimingParameters
                                                .robotRotation
                                                .getRadians()
                                            + Math.PI),
                                    MathUtil.angleModulus(
                                        RobotState.getInstance()
                                            .getFieldToRobot()
                                            .getRotation()
                                            .getRadians()),
                                    Math.toRadians(3))))
                    .withTimeout(1), // how much of a timeout?
                Commands.deadline(Commands.waitSeconds(scoreTime), new FiringCommand())),
            // state; feeder; aiming drive command
            Commands.sequence(
                superstructure.setStateCommand(SuperstructureState.SHOOTING),
                Commands.parallel( // how to end?
                    feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                    aimingDriveCommand(target))))
        .finallyDo(() -> superstructure.setCurrentState(SuperstructureState.TRENCH));
  }

  // confirmed score method, no actuation required
  // public Command simpleScore(TargetType target, double scoreTime) {
  //   return Commands.sequence(
  //           // setup -> state & spin up shooter
  //           superstructure.setStateCommand(SuperstructureState.SHOOTING),
  //           Commands.deadline(
  //               Commands.sequence(
  //                   Commands.deadline(
  //                       Commands.sequence(
  //                           Commands.waitSeconds(1),
  //                           Commands.waitUntil(
  //                           () -> shooter.isAtGoalVelocity(RotationsPerSecond.of(5)))
  //                           .withTimeout(2)),
  //                       feeder.runAtVelocityCommand(RotationsPerSecond.of(85))),
  //                   // shooting
  //                   Commands.deadline(
  //                       Commands.waitSeconds(scoreTime),
  //                       firingCommand())),
  //               aimingDriveCommand(target)))
  //       .finallyDo(() -> superstructure.setCurrentState(SuperstructureState.TRENCH));
  // }

  Pair<Pose2d, Command> Depot() {
    return Pair.of(
        getStartPose(ChorPaths.DEPOT_PICKUP),
        Commands.sequence(
            intakePivot.setCommand(IntakePosition.DEPOT_POSITION),
            Commands.deadline(
                followPathCommand(ChorPaths.DEPOT_PICKUP),
                IntakeRollerSubsystem.getInstance().runIntakeCommand()),
            followPathCommand(ChorPaths.DEPOT_SCORE),
            scoreWhileActuatingHalfway(TargetType.HUB, 2.5),
            intakePivot.setCommand(IntakePosition.OUT_POSITION),
            followPathCommand(ChorPaths.DEPOT_RETURN)));
  }

  // actuates intake
  public Command scoreWhileActuating(TargetType target, double scoreTime) {
    return Commands.deadline(
        simpleScore(target, scoreTime),
        Commands.sequence(
            // Commands.waitSeconds(2), // amount of pause time before actuating
            Commands.parallel(
                IntakeRollerSubsystem.getInstance().runIntakeCommand(),
                Commands.repeatingSequence(
                    IntakePivotSubsystem.getInstance().setCommand(IntakePosition.IN_POSITION),
                    Commands.waitSeconds(0.5),
                    IntakePivotSubsystem.getInstance().setCommand(IntakePosition.OUT_POSITION),
                    Commands.waitSeconds(0.5)))));
  }

  // actuates intake
  // TODO: test if we should keep the intake all the way up
  public Command scoreWhileActuatingHalfway(TargetType target, double scoreTime) {
    return Commands.deadline(
        simpleScore(target, scoreTime),
        Commands.parallel(
            // IntakeRollerSubsystem.getInstance().runIntakeCommand(),
            Commands.sequence(
                Commands.waitSeconds(2),
                IntakePivotSubsystem.getInstance().setCommand(IntakePosition.HALFWAY_POSITION))));
  }

  public Command aimingDriveCommand(TargetType type) {
    return new AimingDriveCommand(
        () -> 0, () -> 0, () -> 0, () -> true, () -> false, () -> false, Optional.of(type));
  }

  // hood
  public Command forceHoodDown() {
    return Commands.runOnce(() -> hood.setCommand(HoodConstants.HOOD_MIN_ANGLE));
  }

  // following / paths
  public Command followPathCommand(ChorPaths chorPath) {
    Tracer tracer = new Tracer();
    // go from a ChorPath String to a PathPlannerPath
    try {
      tracer.addEpoch("startFollowPathCommand");
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      path.preventFlipping = false;
      tracer.addEpoch("followPath - before return");
      Command ret = AutoBuilder.followPath(path);

      tracer.addEpoch("followPath done");
      tracer.printEpochs();

      return ret;
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return Commands.none();
    }
  }

  static PathPlannerPath getOutpostPath(ChorPaths chorPath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return path;
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return null;
    }
  }

  static Pose2d getRedStartPose(ChorPaths chorPath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return path.getStartingHolonomicPose().get();
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return new Pose2d();
    }
  }

  static Pose2d staticStartPose(ChorPaths chorPath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return getAllianceAdjustedPose(path.getStartingHolonomicPose().get());
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return new Pose2d();
    }
  }

  public Pose2d getStartPose(ChorPaths chorPath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return getAllianceAdjustedPose(path.getStartingHolonomicPose().get());
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return new Pose2d();
    }
  }
}
// spotless: on
