// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.input.T16000MJoystick;
import com.team2052.lib.vision.questnav.QuestNavSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FiringCommand;
import frc.robot.commands.drive.AimingDriveCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.drive.DrivetrainConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakePosition;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.FieldRegion;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;
import frc.robot.subsystems.superstructure.shotTables.HubShootingTable;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.MatchState;
import java.util.Optional;

public class RobotContainer {
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  public final VisionSubsystem vision = VisionSubsystem.getInstance();
  public final FeederSubsystem feeder = FeederSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final StagerSubsystem stager = StagerSubsystem.getInstance();
  //   public final ExtendingHopperSubsystem hopper = ExtendingHopperSubsystem.getInstance();
  public final IntakeRollerSubsystem intakeRoller = IntakeRollerSubsystem.getInstance();
  public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
  public final Superstructure superstructure = Superstructure.getInstance();
  public final HoodSubsystem hood = HoodSubsystem.getInstance();
  public final QuestNavSubsystem questNav = new QuestNavSubsystem(new Transform3d());
  // public final LEDSubsystem leds = LEDSubsystem.getInstance();

  public final T16000MJoystick translationJoystick = new T16000MJoystick(0);
  public final T16000MJoystick rotationJoystick = new T16000MJoystick(1);

  public final CommandJoystick secondaryPanel = new CommandJoystick(2);

  public RobotContainer() {
    configureMatchBindings();
    // configureTestBindings();
  }

  private void configureMatchBindings() {
    /*
     *  Primary Controls
     */

    // ---- ODOMETRY RESETS ----

    // reset gyro
    translationJoystick
        .rightBaseBottomLeft()
        .onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

    // reset position to left corner of the field (from driver station perspective)
    translationJoystick
        .rightBaseBottomMiddle()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            MatchState.isRedAlliance()
                                ? DrivetrainConstants.SEED_LEFT_RED_TRANSLATION
                                : DrivetrainConstants.SEED_LEFT_BLUE_TRANSLATION,
                            drivetrain.getState().Pose.getRotation()))));

    // reset position to right corner of the field (from driver station perspective)
    translationJoystick
        .rightBaseBottomRight()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            MatchState.isRedAlliance()
                                ? DrivetrainConstants.SEED_RIGHT_RED_TRANSLATION
                                : DrivetrainConstants.SEED_RIGHT_BLUE_TRANSLATION,
                            drivetrain.getState().Pose.getRotation()))));

    // reset to vision pose
    translationJoystick
        .rightBaseTopMiddle()
        .onTrue(
            new InstantCommand(
                () -> drivetrain.resetPose(RobotState.getInstance().getChassisVisionFieldPose())));

    // ---- PRIMARY CONTROLS ----

    // drive command
    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            translationJoystick::getY,
            translationJoystick::getX,
            rotationJoystick::getX,
            () -> true));

    // run intake and push pivot out
    translationJoystick
        .frontTrigger()
        .whileTrue(intakeRoller.runIntakeCommand())
        .onTrue(
            Commands.either(
                intakePivot.setCommand(IntakePosition.DEPOT_POSITION),
                intakePivot.setCommand(IntakePosition.OUT_POSITION),
                translationJoystick.rightThumbButton()));

    // fire command
    translationJoystick.middleThumbButton().whileTrue(new FiringCommand());

    rotationJoystick
        .frontTrigger()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () ->
                        superstructure.setManualShootingParameters(
                            new Pair<>(
                                ShooterConstants.IDLE_VELOCITY, HoodConstants.HOOD_MIN_ANGLE))),
                superstructure.setStateCommand(SuperstructureState.MANUAL)))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (superstructure.getCurrentState().equals(SuperstructureState.MANUAL)) {
                    superstructure.setCurrentState(SuperstructureState.TRENCH);
                  }
                }));

    // override target to depot feeding
    rotationJoystick
        .leftThumbButton()
        .whileTrue(
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                Commands.sequence(
                    superstructure.overrideTargetCommand(TargetType.DEPOT_FEEDING),
                    new AimingDriveCommand(
                        translationJoystick::getY,
                        translationJoystick::getX,
                        rotationJoystick::getX,
                        () -> true, // field centric
                        () -> true, // use SOTM
                        () -> false, // lock wheels
                        Optional.of(TargetType.OUTPOST_FEEDING))),
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () ->
                            (shooter.isAtGoalVelocity(
                                RotationsPerSecond.of(
                                    superstructure
                                            .getCurrentFieldRegion()
                                            .equals(FieldRegion.ALLIANCE_ZONE)
                                        ? 1
                                        : 5)))),
                    Commands.waitSeconds(0.2),
                    Commands.parallel(new FiringCommand(), compressHopperCommand()))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // override target to outpost feeding
    rotationJoystick
        .rightThumbButton()
        .whileTrue(
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                Commands.sequence(
                    superstructure.overrideTargetCommand(TargetType.OUTPOST_FEEDING),
                    new AimingDriveCommand(
                        translationJoystick::getY,
                        translationJoystick::getX,
                        rotationJoystick::getX,
                        () -> true, // field centric
                        () -> true, // use SOTM
                        () -> false, // lock wheels
                        Optional.of(TargetType.OUTPOST_FEEDING))),
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.waitUntil(
                        () ->
                            (shooter.isAtGoalVelocity(
                                RotationsPerSecond.of(
                                    superstructure
                                            .getCurrentFieldRegion()
                                            .equals(FieldRegion.ALLIANCE_ZONE)
                                        ? 1
                                        : 5)))),
                    Commands.waitSeconds(0.2),
                    Commands.parallel(new FiringCommand(), compressHopperCommand()))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    rotationJoystick
        .middleThumbButton()
        .whileTrue(scoringSequenceCommand())
        .onFalse(
            Commands.parallel(
                superstructure.setStateCommand(SuperstructureState.TRENCH),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.parallel(feeder.runAtPctCommand(-0.5), stager.runAtPctCommand(-0.5))
                        .withTimeout(0.5))));
    secondaryPanel
        .button(9)
        .whileTrue(scoringSequenceCommand())
        .onFalse(
            Commands.parallel(
                superstructure.setStateCommand(SuperstructureState.TRENCH),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.parallel(feeder.runAtPctCommand(-0.5), stager.runAtPctCommand(-0.5))
                        .withTimeout(0.5))));
    // .whileTrue(
    //     Commands.sequence(
    //         superstructure.setStateCommand(SuperstructureState.SHOOTING),
    //         Commands.parallel(
    //             feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
    //             new AimingDriveCommand(
    //                 translationJoystick::getY,
    //                 translationJoystick::getX,
    //                 rotationJoystick::getX,
    //                 () -> true, // field centric
    //                 () ->
    //                     !superstructure
    //                         .getCurrentFieldRegion()
    //                         .equals(FieldRegion.ALLIANCE_ZONE), // use SOTM
    //                 () ->
    //                     MathHelpers.angleEpsilonEquals(
    //                             superstructure
    //                                 .getLastCalculatedProfile()
    //                                 .aimingParameters
    //                                 .robotRotation
    //                                 .getMeasure()
    //                                 .plus(Degrees.of(MatchState.isRedAlliance() ? 180 : 0)),
    //                             RobotState.getInstance()
    //                                 .getFieldToRobot()
    //                                 .getRotation()
    //                                 .getMeasure(),
    //                             Degrees.of(3))
    //                         && superstructure
    //                             .getCurrentFieldRegion()
    //                             .equals(FieldRegion.ALLIANCE_ZONE), // lock wheels
    //                 Optional.empty() // default target type
    //                 ))))
    // .whileTrue(
    //     Commands.sequence(
    //         Commands.waitSeconds(0.5),
    //         Commands.waitUntil(
    //             () ->
    //                 (shooter.isAtGoalVelocity(
    //                         RotationsPerSecond.of(
    //                             superstructure
    //                                     .getCurrentFieldRegion()
    //                                     .equals(FieldRegion.ALLIANCE_ZONE)
    //                                 ? 1
    //                                 : 5))
    //                     && MathHelpers.angleEpsilonEquals(
    //                         superstructure
    //                             .getLastCalculatedProfile()
    //                             .aimingParameters
    //                             .robotRotation
    //                             .getMeasure()
    //                             .plus(Degrees.of(MatchState.isRedAlliance() ? 180 : 0)),
    //                         RobotState.getInstance()
    //                             .getFieldToRobot()
    //                             .getRotation()
    //                             .getMeasure(),
    //                         Degrees.of(3)))), // .withTimeout(2),
    //         Commands.waitSeconds(0.2),
    //         new FiringCommand()))

    translationJoystick
        .leftThumbButton()
        .whileTrue(
            Commands.parallel(
                intakeRoller.runOuttakeCommand(),
                floor.runAtVelocityCommand(RotationsPerSecond.of(-50)),
                stager.runAtVelocityCommand(RotationsPerSecond.of(-50)),
                feeder.runAtVelocityCommand(RotationsPerSecond.of(-50)),
                intakePivot.setCommand(IntakePosition.OUT_POSITION)));

    // aim to default target

    // ---- SECONDARY PANEL CONTROLS ----

    // side buttons
    secondaryPanel.button(2).onFalse(intakePivot.setCommand(IntakePosition.STOW_POSITION));
    Trigger halfwayIntake = new Trigger(() -> secondaryPanel.getX() < 0.5);
    halfwayIntake.onFalse(intakePivot.setCommand(IntakePosition.HALFWAY_POSITION));
    Trigger intakeDown = new Trigger(() -> secondaryPanel.getX() > 0.5);
    intakeDown.onFalse(intakePivot.setCommand(IntakePosition.OUT_POSITION));

    // stow intake
    secondaryPanel.button(12).onFalse(intakePivot.setCommand(IntakePosition.STOW_POSITION));

    // halfway intake
    secondaryPanel.button(5).onFalse(intakePivot.setCommand(IntakePosition.HALFWAY_POSITION));

    // intake out
    secondaryPanel.button(11).onFalse(intakePivot.setCommand(IntakePosition.OUT_POSITION));

    // set hood to min angle
    secondaryPanel.button(10).onFalse(compressHopperCommand());

    secondaryPanel
        .button(3)
        .onFalse(Commands.runOnce(() -> superstructure.increaseBrownoutBoost()));
    secondaryPanel
        .button(4)
        .onFalse(Commands.runOnce(() -> superstructure.decreaseBrownoutBoost()));

    // RIGHT TOWER
    secondaryPanel
        .button(8)
        .whileTrue(
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            superstructure.setManualShootingParameters(
                                //         new Pair<>(RotationsPerSecond.of(0), Degrees.of(25)))),
                                HubShootingTable.getInstance()
                                    .getShootingParameters(Meters.of(3.914)))),
                    superstructure.setStateCommand(SuperstructureState.MANUAL))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // LEFT TOWER
    secondaryPanel
        .button(7)
        .onTrue(
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            superstructure.setManualShootingParameters(
                                HubShootingTable.getInstance()
                                    .getShootingParameters(Meters.of(3.724)))),
                    superstructure.setStateCommand(SuperstructureState.MANUAL))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // LEFT TRENCH
    Trigger leftTrenchTrigger = new Trigger(() -> secondaryPanel.getY() < -0.5);
    leftTrenchTrigger
        .onTrue(
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            superstructure.setManualShootingParameters(
                                HubShootingTable.getInstance()
                                    .getShootingParameters(Meters.of(3.372)))),
                    superstructure.setStateCommand(SuperstructureState.MANUAL))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // RIGHT TRENCH
    // secondaryPanel
    //     .button(9)
    //     .onTrue(
    //         Commands.parallel(
    //             feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
    //             Commands.sequence(
    //                 Commands.runOnce(
    //                     () ->
    //                         superstructure.setManualShootingParameters(
    //                             HubShootingTable.getInstance()
    //                                 .getShootingParameters(Meters.of(3.372)))),
    //                 superstructure.setStateCommand(SuperstructureState.MANUAL))))
    //     .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));
  }

  //   private void configureTestBindings() {
  //     //     /*  How to run these tests WITHOUT having logger:
  //     //      * 1. make sure this method is called instead of configureMatchBindings() in the
  //     // constructor
  //     //      * 2. setup the hood first cause otherwise it will try to go to the min angle and if
  //     // thats
  //     // not setup it will break.
  //     //      * 3. note that nothing should run on enable, so you can test everything
  // individually
  //     // before switching to the match bindings
  //     //      * 4. check all the button bindings cause I put some of them (mostly secondary panel
  //     // ones)
  //     // on random numbers cause IDK which to use.
  //     //      * 5. Just follow what each thing says to do, you'll be fine. Text me if you don't
  // know
  //     // how
  //     // anything works.
  //     //      */

  //     // DRIVETRAIN TESTS

  //     // reset gyro
  //     translationJoystick
  //         .rightBaseBottomLeft()
  //         .onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));
  //     // drive command
  //     drivetrain.setDefaultCommand(
  //         new DefaultDriveCommand(
  //             translationJoystick::getY,
  //             translationJoystick::getX,
  //             rotationJoystick::getX,
  //             () -> true));
  //     translationJoystick
  //         .middleThumbButton()
  //         .whileTrue(Commands.parallel(new FiringCommand(), compressHopperCommand()));
  //     rotationJoystick
  //         .frontTrigger()
  //         .onTrue(feeder.runAtVelocityCommand(RotationsPerSecond.of(85)))
  //         .onFalse(feeder.runAtVelocityCommand(RotationsPerSecond.of(0)));

  //     rotationJoystick
  //         .middleThumbButton()
  //         .whileTrue(
  //             Commands.sequence(
  //                 superstructure.setStateCommand(SuperstructureState.SHOOTING),
  //                 Commands.parallel(
  //                     feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
  //                     new AimingDriveCommand(
  //                         translationJoystick::getY,
  //                         translationJoystick::getX,
  //                         rotationJoystick::getX,
  //                         () -> true, // field centric
  //                         () ->
  //                             !superstructure
  //                                 .getCurrentFieldRegion()
  //                                 .equals(FieldRegion.ALLIANCE_ZONE), // use SOTM
  //                         () -> false,
  //                         //   MathHelpers.epsilonEquals(
  //                         //           MathUtil.angleModulus(
  //                         //               superstructure
  //                         //                       .getLastCalculatedProfile()
  //                         //                       .aimingParameters
  //                         //                       .robotRotation
  //                         //                       .getRadians()
  //                         //                   + Math.PI),
  //                         //           MathUtil.angleModulus(
  //                         //               RobotState.getInstance()
  //                         //                   .getFieldToRobot()
  //                         //                   .getRotation()
  //                         //                   .getRadians()),
  //                         //           Math.toRadians(3))
  //                         //   && superstructure
  //                         //       .getCurrentFieldRegion()
  //                         //       .equals(FieldRegion.ALLIANCE_ZONE), // lock wheels
  //                         Optional.empty() // default target type
  //                         ))))
  //         //   .whileTrue(
  //         //       Commands.sequence(
  //         //           Commands.waitSeconds(0.5),
  //         //           Commands.waitUntil(
  //         //               () ->
  //         //                   (shooter.isAtGoalVelocity(RotationsPerSecond.of(1))
  //         //                     //   && MathHelpers.epsilonEquals(
  //         //                     //       MathUtil.angleModulus(
  //         //                     //           superstructure
  //         //                     //                   .getLastCalculatedProfile()
  //         //                     //                   .aimingParameters
  //         //                     //                   .robotRotation
  //         //                     //                   .getRadians()
  //         //                     //               + Math.PI),
  //         //                     //       MathUtil.angleModulus(
  //         //                     //           RobotState.getInstance()
  //         //                     //               .getFieldToRobot()
  //         //                     //               .getRotation()
  //         //                     //               .getRadians()),
  //         //                           Math.toRadians(3)))), // .withTimeout(2),
  //         //           Commands.waitSeconds(0.2),
  //         //           new FiringCommand()))
  //         .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));
  //     translationJoystick
  //         .frontTrigger()
  //         .whileTrue(intakeRoller.runIntakeCommand())
  //         .onTrue(intakePivot.setCommand(IntakePosition.OUT_POSITION));
  //     // stow intake
  //     secondaryPanel.button(12).onFalse(intakePivot.setCommand(IntakePosition.STOW_POSITION));

  //     // halfway intake
  //     secondaryPanel.button(5).onFalse(intakePivot.setCommand(IntakePosition.DEPOT_POSITION));

  //     // intake out
  //     secondaryPanel.button(11).onFalse(intakePivot.setCommand(IntakePosition.OUT_POSITION));

  //     // plus minus half a degree on the hood
  //     secondaryPanel
  //         .button(1)
  //         .onFalse(Commands.runOnce(() -> hood.set(hood.getGoalAngle().plus(Degrees.of(0.5)))));
  //     secondaryPanel
  //         .button(6)
  //         .onFalse(Commands.runOnce(() -> hood.set(hood.getGoalAngle().minus(Degrees.of(0.5)))));

  //     // // plus minus 1 RPS on the shooter
  //     secondaryPanel
  //         .button(3)
  //         .onFalse(
  //             Commands.runOnce(
  //                 () ->
  //                     shooter.setGoalPoint(
  //                         shooter.getGoalPoint().plus(RotationsPerSecond.of(0.25)))));
  //     secondaryPanel
  //         .button(4)
  //         .onFalse(
  //             Commands.runOnce(
  //                 () ->
  //                     shooter.setGoalPoint(
  //                         shooter.getGoalPoint().minus(RotationsPerSecond.of(0.25)))));

  //     // // print out the current shot profile
  //     secondaryPanel
  //         .button(10)
  //         .onFalse(
  //             Commands.runOnce(
  //                 () -> {
  //                   System.out.println(
  //                       "Current hood angle: " + hood.getGoalAngle().in(Degrees) + " degrees");
  //                   System.out.println(
  //                       "Current shooter velocity: "
  //                           + shooter.getGoalPoint().in(RotationsPerSecond)
  //                           + " RPS");
  //                 }));
  //     secondaryPanel
  //         .button(10)
  //         .onFalse(
  //             Commands.defer(
  //                 () ->
  //                     Commands.print(
  //                         "Distance To Hub: "
  //                             + RobotState.getInstance()
  //                                 .getFieldToRobot()
  //                                 .getTranslation()
  //                                 .getDistance(
  //                                     FieldConstants.FieldLocations.RED_ALLIANCE_HUB_LOCATION)),
  //                 Set.of(new Subsystem() {})));
  //     translationJoystick
  //         .leftThumbButton()
  //         .whileTrue(
  //             Commands.parallel(
  //                 intakeRoller.runOuttakeCommand(),
  //                 floor.runAtVelocityCommand(RotationsPerSecond.of(-50)),
  //                 stager.runAtVelocityCommand(RotationsPerSecond.of(-50)),
  //                 feeder.runAtVelocityCommand(RotationsPerSecond.of(-50)),
  //                 intakePivot.setCommand(IntakePosition.OUT_POSITION)));
  //   }

  private Command scoringSequenceCommand() {
    return Commands.parallel(
        Commands.sequence(
            superstructure.setStateCommand(SuperstructureState.SHOOTING),
            Commands.parallel(
                feeder.runAtVelocityCommand(RotationsPerSecond.of(85)),
                new AimingDriveCommand(
                    translationJoystick::getY,
                    translationJoystick::getX,
                    rotationJoystick::getX,
                    () -> true, // field centric
                    () ->
                        !superstructure
                            .getCurrentFieldRegion()
                            .equals(FieldRegion.ALLIANCE_ZONE), // use SOTM
                    () ->
                        MathHelpers.angleEpsilonEquals(
                                superstructure
                                    .getLastCalculatedProfile()
                                    .aimingParameters
                                    .robotRotation
                                    .getMeasure()
                                    .plus(Degrees.of(MatchState.isRedAlliance() ? 180 : 0)),
                                RobotState.getInstance()
                                    .getFieldToRobot()
                                    .getRotation()
                                    .getMeasure(),
                                Degrees.of(3))
                            && superstructure
                                .getCurrentFieldRegion()
                                .equals(FieldRegion.ALLIANCE_ZONE), // lock wheels
                    Optional.empty() // default target type
                    ))),
        Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.waitUntil(
                () ->
                    (shooter.isAtGoalVelocity(
                        RotationsPerSecond.of(
                            superstructure.getCurrentFieldRegion().equals(FieldRegion.ALLIANCE_ZONE)
                                ? 1
                                : 5)))),
            // && MathHelpers.angleEpsilonEquals(
            //     superstructure
            //         .getLastCalculatedProfile()
            //         .aimingParameters
            //         .robotRotation
            //         .getMeasure()
            //         .plus(Degrees.of(MatchState.isRedAlliance() ? 180 : 0)),
            //     RobotState.getInstance().getFieldToRobot().getRotation().getMeasure(),
            //     Degrees.of(3)))), // .withTimeout(2),
            Commands.waitSeconds(0.2),
            // new FiringCommand()));
            Commands.parallel(new FiringCommand(), compressHopperCommand())));
  }

  private Command compressHopperCommand() {
    return Commands.sequence(
        // Commands.waitUntil(() -> !translationJoystick.frontTrigger().getAsBoolean()),
        Commands.deadline(
            intakePivot.compressCommand(),
            intakeRoller.runAtVelocityCommand(IntakeConstants.INTAKE_VELOCITY)));
    // .finallyDo(
    //     () ->
    //         Commands.parallel(
    //             intakeRoller.runAtVelocityCommand(RotationsPerSecond.of(0)),
    //             intakePivot.setCommand(IntakePosition.STOW_POSITION)));
  }
}
