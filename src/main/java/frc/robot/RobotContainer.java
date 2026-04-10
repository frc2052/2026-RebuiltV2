// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team2052.lib.input.T16000MJoystick;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;

public class RobotContainer {
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final FeederSubsystem feeder = FeederSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final StagerSubsystem stager = StagerSubsystem.getInstance();

  public final T16000MJoystick translationJoystick = new T16000MJoystick(0);
  public final T16000MJoystick rotationJoystick = new T16000MJoystick(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    translationJoystick
        .button(1)
        .whileTrue(shooter.runAtVelocityCommand(RotationsPerSecond.of(40)));

    rotationJoystick.button(1).whileTrue(stager.runAtVelocityCommand(RotationsPerSecond.of(80)));
    rotationJoystick
        .button(2)
        .whileTrue(Commands.parallel(floor.runAtVelocityCommand(RotationsPerSecond.of(113))));
    translationJoystick
        .button(2)
        .whileTrue(Commands.parallel(feeder.runAtVelocityCommand(RotationsPerSecond.of(85))));

    rotationJoystick
        .button(3)
        .whileTrue(
            Commands.parallel(
                stager.runAtVelocityCommand(RotationsPerSecond.of(-55)),
                feeder.runAtVelocityCommand(RotationsPerSecond.of(-55)),
                floor.runAtPctCommand(-0.75)));
  }
}
