// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class RobotContainer {
  // public final FloorSubsystem floor = FloorSubsystem.getInstance();
  // public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  // public final VisionSubsystem vision = VisionSubsystem.getInstance();
  // public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  // public final StagerSubsystem stager = StagerSubsystem.getInstance();
  // public final ExtendingHopperSubsystem hopper = ExtendingHopperSubsystem.getInstance();
  // public final IntakePivotSubsystem intake = IntakePivotSubsystem.getInstance();
  // public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();

  // public final T16000MJoystick translationJoystick = new T16000MJoystick(0);
  // public final T16000MJoystick rotationJoystick = new T16000MJoystick(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // translationJoystick
    //     .button(1)
    //     .whileTrue(shooter.runAtVelocityCommand(RotationsPerSecond.of(40)));
    // rotationJoystick.button(1).whileTrue(stager.runAtVelocityCommand(RotationsPerSecond.of(55)));
    // rotationJoystick.button(2).whileTrue(floor.runAtPctCommand(0.75));
    // rotationJoystick
    //     .button(3)
    //     .whileTrue(
    //         Commands.parallel(
    //             stager.runAtVelocityCommand(RotationsPerSecond.of(-55)),
    //             floor.runAtPctCommand(-0.75)));
  }
}
