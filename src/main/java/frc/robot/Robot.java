// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FieldConstants;

public class Robot extends TimedRobot {

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    // Force load this when the robot starts so that it doesn't cause 3 second overruns.
    double initialize = FieldConstants.fieldWidth;
    addPeriodic(VisionSubsystem.getInstance()::visionPeriodic, Constants.VISION_LOOP_PERIOD, Constants.VISION_LOOP_OFFSET);
  }

  @Override
  public void robotPeriodic() {
    // Reset the shot profile calculation flag at the start of each period
    Superstructure.getInstance().setHasCalculatedShotProfileThisPeriod(false);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
