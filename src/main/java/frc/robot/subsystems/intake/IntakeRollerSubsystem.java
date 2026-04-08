// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerSubsystem extends RollerSubsystem {

  private static IntakeRollerSubsystem INSTANCE;

  public static IntakeRollerSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeRollerSubsystem();
    }
    return INSTANCE;
  }

  private IntakeRollerSubsystem() {
    super(IntakeConstants.INTAKE_ROLLER_CONSTANTS);
  }

  public void runIntake() {
    setGoalVelocity(IntakeConstants.INTAKE_VELOCITY);
  }

  public void runOuttake() {
    setGoalVelocity(IntakeConstants.INTAKE_VELOCITY.unaryMinus());
  }

  public Command runIntakeCommand() {
    return runAtVelocityCommand(IntakeConstants.INTAKE_VELOCITY);
  }

  public Command runOuttakeCommand() {
    return runAtVelocityCommand(IntakeConstants.INTAKE_VELOCITY.unaryMinus());
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
