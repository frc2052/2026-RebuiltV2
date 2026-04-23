package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FiringCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakePosition;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

// spotless: off
public class AutoFactory {
  static final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  static final NetworkTable debugTable = networkTables.getTable("autos networktables tab");
  static final DoubleTopic waitTimeTopic = debugTable.getDoubleTopic("waitTime");
  static final DoublePublisher waitTimePublisher = waitTimeTopic.publish();
  static final DoubleSubscriber waitTimeSubscriber = waitTimeTopic.subscribe(0);

  static final LoggedDashboardChooser sprintChooser =
      new LoggedDashboardChooser<Boolean>(
          "Auto Sprint Mode Chooser", new SendableChooser<Boolean>());

    public AutoFactory() {
    sprintChooser.addDefaultOption("NO SPRINT", Boolean.FALSE);
    sprintChooser.addOption("SPRINT", Boolean.TRUE);

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
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> preloadOnlyRight() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> preloadOnlyCenter() {
        return Pair.of(new Pose2d(), Commands.none());
    }

// LEFT AUTOS

    Pair<Pose2d, Command> leftSingleTrenchSweep() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> leftDoubleSweep() {
        return Pair.of(new Pose2d(), Commands.none());
    }

// RIGHT AUTOS
    Pair<Pose2d, Command> rightSingleTrenchSweep() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> rightDoubleSweep() {
        return Pair.of(new Pose2d(), Commands.none());
    }

// CENTER AUTOS
    Pair<Pose2d, Command> centerOutpostDepot() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> centerDepotOutpost() {
        return Pair.of(new Pose2d(), Commands.none());
    }

// example from v1

    //   Pair<Pose2d, Command> leftSweepTrench() {
    //     return Pair.of(
    //         getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
    //         // first cycle
    //         Commands.sequence(
    //             sprintOrScorePreload(),
    //             postScoringCleanup(),
    //             Commands.waitSeconds(0.2), // wait for hood & intake
    //             Commands.deadline(
    //                 Commands.sequence(
    //                     IntakePivotCommandFactory.extend(),
    //                     followPathCommand(ChorPaths.LTRENCH_LNEUTRAL1),
    //                     followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
    //                 IntakeRollerCommandFactory.intake()),
    //             scoreWithTimeHalfway(5.5),
    //             postScoringCleanup()));
    //   }

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

// other
public static Pose2d getAllianceAdjustedPose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FlippingUtil.flipFieldPose(bluePose);
    } else {
      return bluePose;
    }
  }

  public Command sprintOrScorePreload() {
    return Commands.defer(
        () -> {
          if (sprintChooser.get().equals(Boolean.TRUE)) {
            return Commands.none();
          } else {
            return simpleScoreWithTime(2);
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

    public Command reserGyro(){
        return Commands.runOnce(() -> drivetrain.seedFieldCentric());
    }

    public Command setState(SuperstructureState state){
        return superstructure.setStateCommand(state);
    }

// intaking
    public Command intakeWithTime(double seconds){
        return Commands.sequence(
            intakePivot.setCommand(IntakePosition.OUT_POSITION),
            intake.runIntakeCommand().withTimeout(seconds)
        );
    }

    public Command indexAll(){
        return Commands.runOnce(
            () -> floor.scoreCommand()
        );
    }

// shooting

    public Command postScoringCleanup(){
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.getInstance().setCurrentState(SuperstructureState.TRENCH)),
            Commands.runOnce(() -> intakePivot.setCommand(IntakePosition.OUT_POSITION))
        );
    }

    public Command firingCommand(){
        return new FiringCommand();
    }

    // TODO: ?? do i override the target?
    public Command hubScoringState(){
        return Commands.runOnce(() -> superstructure.setStateCommand(SuperstructureState.SHOOTING));
    }

    public Command trenchScoringState(){
        return Commands.runOnce(() -> superstructure.setStateCommand(SuperstructureState.TRENCH));
    }

    // TODO: do i just set to a scoring state? then run the intake or how do i do this
    public Command socreWithTimeHalfway(double scoreTime){
        return Commands.deadline(
        Commands.waitSeconds(scoreTime),
        Commands.sequence(
            // superstructure.overrideTarget(TargetType.),
            hubScoringState(),
            Commands.waitSeconds(2),
            Commands.parallel(
                Commands.run(() -> IntakeRollerSubsystem.getInstance().runIntake()),
                Commands.repeatingSequence(
                    intakePivot.setCommand(IntakePosition.HALFWAY_POSITION),
                    Commands.waitSeconds(0.5),
                    intakePivot.setCommand(IntakePosition.HALFWAY_POSITION),
                    Commands.waitSeconds(0.5))))); 
    }

    public Command scoreWithTime(double scoreTime){
        return Commands.deadline(
        Commands.waitSeconds(scoreTime),
        Commands.sequence(
            // superstructure.overrideTarget(TargetType.),
            hubScoringState(),
            Commands.waitSeconds(2),
            Commands.parallel(
                Commands.run(() -> IntakeRollerSubsystem.getInstance().runIntake()),
                Commands.repeatingSequence(
                    intakePivot.setCommand(IntakePosition.IN_POSITION),
                    Commands.waitSeconds(0.5),
                    intakePivot.setCommand(IntakePosition.IN_POSITION),
                    Commands.waitSeconds(0.5))))); 
    }

    public Command simpleScoreWithTime(double scoreTime){
      return Commands.deadline(
            Commands.waitSeconds(scoreTime),
            Commands.sequence(
                hubScoringState(),
                Commands.waitSeconds(0.5)))
        .finallyDo(() -> postScoringCleanup());
    }

// hood
    public Command forceHoodDown(){
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