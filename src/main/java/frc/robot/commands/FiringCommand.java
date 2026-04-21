// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.floor.FloorSubsystem;
import frc.robot.subsystems.stager.StagerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FiringCommand extends Command {

  private FloorSubsystem floor = FloorSubsystem.getInstance();
  private FeederSubsystem feeder = FeederSubsystem.getInstance();
  private StagerSubsystem stager = StagerSubsystem.getInstance();

  private Time timeout = null;
  private Timer timer = new Timer();

  public FiringCommand() {

    addRequirements(floor, stager, feeder);
  }

  public FiringCommand(Time timeout) {
    this();
    this.timeout = timeout;
    this.timer.start();
  }

  @Override
  public void initialize() {
    floor.runAtFiringVelocity();
    stager.runAtFiringVelocity();
    feeder.runAtFiringVelocity();
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floor.stopMotor();
    stager.stopMotor();
    feeder.stopMotor();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeout != null) {
      return timer.hasElapsed(timeout);
    }
    return false;
  }
}
