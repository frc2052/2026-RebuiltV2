// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team2052.lib.input.T16000MJoystick;

import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;

public class RobotContainer {
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final StagerSubsystem stager = StagerSubsystem.getInstance();

  public final T16000MJoystick translationJoystick = new T16000MJoystick(0);
  public final T16000MJoystick rotationJoystick = new T16000MJoystick(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    translationJoystick.button(1).whileTrue(shooter.runAtPctCommand(0.1));
  }
}
