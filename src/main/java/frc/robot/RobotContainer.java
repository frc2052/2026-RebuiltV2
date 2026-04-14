// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team2052.lib.input.T16000MJoystick;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.hopper.ExtendingHopperSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  public final VisionSubsystem vision = VisionSubsystem.getInstance();
  public final FeederSubsystem feeder = FeederSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final StagerSubsystem stager = StagerSubsystem.getInstance();
  public final ExtendingHopperSubsystem hopper = ExtendingHopperSubsystem.getInstance();
  public final IntakePivotSubsystem intake = IntakePivotSubsystem.getInstance();
  public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();

  public final T16000MJoystick translationJoystick = new T16000MJoystick(0);
  public final T16000MJoystick rotationJoystick = new T16000MJoystick(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    translationJoystick
        .button(1)
        .whileTrue(shooter.runAtVelocityCommand(RotationsPerSecond.of(35)));

    // rotationJoystick.button(1).whileTrue(stager.runAtVelocityCommand(RotationsPerSecond.of(80)));
    rotationJoystick
        .button(2)
        .whileTrue(
            Commands.sequence(
                // Commands.parallel(
                //         floor.runAtVelocityCommand(RotationsPerSecond.of(-60)),
                //         stager.runAtVelocityCommand(RotationsPerSecond.of(60)))
                //     .withTimeout(Seconds.of(0.5)),
                Commands.parallel(
                    floor.runAtVelocityCommand(RotationsPerSecond.of(60)),
                    stager.runAtVelocityCommand(RotationsPerSecond.of(55)))));
    translationJoystick
        .button(2)
        .whileTrue(Commands.parallel(feeder.runAtVelocityCommand(RotationsPerSecond.of(96))));

    rotationJoystick
        .button(3)
        .whileTrue(
            Commands.parallel(
                stager.runAtVelocityCommand(RotationsPerSecond.of(-55)),
                feeder.runAtVelocityCommand(RotationsPerSecond.of(-55)),
                floor.runAtPctCommand(-0.75)));
  }
}
