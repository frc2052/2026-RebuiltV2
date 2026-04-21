// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team2052.lib.input.T16000MJoystick;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.hopper.ExtendingHopperSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakePosition;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;
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
  public final ExtendingHopperSubsystem hopper = ExtendingHopperSubsystem.getInstance();
  public final IntakeRollerSubsystem intake = IntakeRollerSubsystem.getInstance();
  public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
  public final Superstructure superstructure = Superstructure.getInstance();
  public final HoodSubsystem hood = HoodSubsystem.getInstance();

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
        .whileTrue(intake.runIntakeCommand())
        .onTrue(intakePivot.setCommand(IntakePosition.OUT_POSITION));

    rotationJoystick
        .frontTrigger()
        .whileTrue(shooter.runAtVelocityCommand(RotationsPerSecond.of(30)));

    // fire command
    translationJoystick.middleThumbButton().whileTrue(new FiringCommand());

    // override target to depot feeding
    rotationJoystick
        .leftThumbButton()
        .whileTrue(
            Commands.sequence(
                superstructure.overrideTargetCommand(TargetType.DEPOT_FEEDING),
                new AimingDriveCommand(
                    translationJoystick::getY,
                    translationJoystick::getX,
                    rotationJoystick::getX,
                    () -> true, // field centric
                    () -> true, // use SOTM
                    () -> false, // lock wheels
                    Optional.of(TargetType.DEPOT_FEEDING))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // override target to outpost feeding
    rotationJoystick
        .rightThumbButton()
        .whileTrue(
            Commands.sequence(
                superstructure.overrideTargetCommand(TargetType.OUTPOST_FEEDING),
                new AimingDriveCommand(
                    translationJoystick::getY,
                    translationJoystick::getX,
                    rotationJoystick::getX,
                    () -> true, // field centric
                    () -> true, // use SOTM
                    () -> false, // lock wheels
                    Optional.of(TargetType.OUTPOST_FEEDING))))
        .onFalse(superstructure.setStateCommand(SuperstructureState.TRENCH));

    // aim to default target
    rotationJoystick
        .middleThumbButton()
        .whileTrue(
            Commands.sequence(
                superstructure.setStateCommand(SuperstructureState.SHOOTING),
                new AimingDriveCommand(
                    translationJoystick::getY,
                    translationJoystick::getX,
                    rotationJoystick::getX,
                    () -> true, // field centric
                    () -> false, // use SOTM
                    () -> translationJoystick.middleThumbButton().getAsBoolean(), // lock wheels
                    Optional.empty() // default target type
                    )));

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
    secondaryPanel.button(10).onFalse(hood.setCommand(HoodConstants.HOOD_MIN_ANGLE));

    // set hood to max angle
    secondaryPanel.button(1).onFalse(hood.setCommand((HoodConstants.HOOD_MAX_ANGLE)));

    // set hood to halfway degrees
    secondaryPanel.button(6).onFalse(hood.setCommand(Degrees.of(15)));
  }

  private void configureTestBindings() {
    /*  How to run these tests WITHOUT having logger:
     * 1. make sure this method is called instead of configureMatchBindings() in the constructor
     * 2. setup the hood first cause otherwise it will try to go to the min angle and if thats not setup it will break.
     * 3. note that nothing should run on enable, so you can test everything individually before switching to the match bindings
     * 4. check all the button bindings cause I put some of them (mostly secondary panel ones) on random numbers cause IDK which to use.
     * 5. Just follow what each thing says to do, you'll be fine. Text me if you don't know how anything works.
     */

    // DRIVETRAIN TESTS

    // drive command
    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            translationJoystick::getY,
            translationJoystick::getX,
            rotationJoystick::getX,
            () -> true));

    // aiming command with no SOTM and point to hub,
    // use this to test limelight stuff and PID tuning.
    // you should tune the PID with a full hopper cause 40 lbs on balls will affect the tuning
    // significantly.
    // also see how effective the locked wheels are.
    rotationJoystick
        .middleThumbButton()
        .whileTrue(
            new AimingDriveCommand(
                translationJoystick::getY,
                translationJoystick::getX,
                rotationJoystick::getX,
                () -> true, // field centric
                () -> false, // use SOTM
                () -> rotationJoystick.frontTrigger().getAsBoolean(), // lock wheels
                Optional.of(TargetType.HUB)));

    // INTAKE TESTS

    // idk just do the PID stuff and make sure the positions are correct.
    // also test the rollers (make sure the speeds are correct this time)
    // also maybe see if how fast we can drive while still intaking and not beaching? that would be
    // good to know.

    // stow intake
    secondaryPanel.button(12).onFalse(intakePivot.setCommand(IntakePosition.STOW_POSITION));

    // halfway intake
    secondaryPanel.button(5).onFalse(intakePivot.setCommand(IntakePosition.HALFWAY_POSITION));

    // intake out
    secondaryPanel.button(11).onFalse(intakePivot.setCommand(IntakePosition.OUT_POSITION));

    // run intake rollers
    translationJoystick.frontTrigger().whileTrue(intake.runIntakeCommand());

    // HOOD TESTS

    // YOU NEED TO COMMENT OUT SUPERSTRUCTURE FOR THIS CAUSE OTHERWISE IT WILL JUST SET IT TO THE
    // MIN ANGLE!!!!

    // set hood to min angle
    secondaryPanel.button(10).onFalse(hood.setCommand(HoodConstants.HOOD_MIN_ANGLE));

    // set hood to max angle
    secondaryPanel.button(1).onFalse(hood.setCommand((HoodConstants.HOOD_MAX_ANGLE)));

    // set hood to halfway degrees
    secondaryPanel.button(6).onFalse(hood.setCommand(Degrees.of(22.5)));

    // SCORING THINGS

    // just make sure they still work

    // run the shooter at whatever speed you input here.

    translationJoystick
        .leftThumbButton()
        .whileTrue(Commands.runOnce(() -> shooter.setGoalVelocity(RotationsPerSecond.of(40))));

    // fire command, this runs the floor, feeder, and stager
    translationJoystick.middleThumbButton().whileTrue(new FiringCommand());

    // HOPPER TESTS

    // do PID and positions. also do falling stuff and make sure it stops and runs at the right
    // speeds.

    // side buttons
    // secondaryPanel
    //     .button(2)
    //     .onTrue(hopper.setStateCommand(ExtendingHopperSubsystem.HopperState.RETRACTED));
    // Trigger hopperRetract = new Trigger(() -> secondaryPanel.getX() < 0.5);
    // hopperRetract.onFalse(hopper.setStateCommand(ExtendingHopperSubsystem.HopperState.RETRACTED));
    // Trigger hopperFall = new Trigger(() -> secondaryPanel.getX() > 0.5);
    // hopperFall.onFalse(hopper.setStateCommand(ExtendingHopperSubsystem.HopperState.FALLING));

    // ok so if you've gotten this far GREAT! I didn't think you'd be able to get everything working
    // without me ;)
    // so... idk here are the other things, you should prob switch to match bindings and make sure
    // those work but here is where I'ma setup the shot profile tuning stuff.

    // plus minus half a degree on the hood
    // secondaryPanel
    //     .button(4)
    //     .onFalse(
    //         Commands.runOnce(() -> hood.setToAngle(hood.getGoalAngle().plus(Degrees.of(0.5)))));
    // secondaryPanel
    //     .button(6)
    //     .onFalse(
    //         Commands.runOnce(() -> hood.setToAngle(hood.getGoalAngle().minus(Degrees.of(0.5)))));

    // // plus minus 1 RPS on the shooter
    // secondaryPanel
    //     .button(7)
    //     .onFalse(
    //         Commands.runOnce(
    //             () ->
    //                 shooter.setGoalVelocity(
    //                     shooter.getGoalPoint().plus(RotationsPerSecond.of(1)))));
    // secondaryPanel
    //     .button(8)
    //     .onFalse(
    //         Commands.runOnce(
    //             () ->
    //                 shooter.setGoalVelocity(
    //                     shooter.getGoalPoint().minus(RotationsPerSecond.of(1)))));

    // // print out the current shot profile
    // secondaryPanel
    //     .button(9)
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               System.out.println(
    //                   "Current hood angle: " + hood.getGoalAngle().in(Degrees) + " degrees");
    //               System.out.println(
    //                   "Current shooter velocity: "
    //                       + shooter.getGoalPoint().in(RotationsPerSecond)
    //                       + " RPS");
    //             }));
  }
}
