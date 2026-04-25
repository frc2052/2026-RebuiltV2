// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

// AutoChooser manages letting us choose autos before a match & sets start poses
public class AutoChooser {

  private final AutoFactory factory;
  private Autos lastSelected = null;
  private Pair<Pose2d, Command> currentAuto = null;
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public AutoChooser(RobotContainer robotContainer) {

    factory = new AutoFactory();

    currentAuto = Autos.NO_AUTO.getStartPoseAndRoutine(factory);
    lastSelected = Autos.PRELOAD_ONLY_LEFT;

    // go through ALL AutoPrograms and put them on the Chooser
    for (Autos auto : Autos.values()) {
      if (auto.equals(Autos.PRELOAD_ONLY_LEFT)) {
        autoChooser.setDefaultOption(auto.name(), auto);
        System.out.println("Added default Auto: " + auto.name());
      } else {
        autoChooser.addOption(auto.name(), auto);
        System.out.println("Added auto option: " + auto.name());
      }
    }

    SmartDashboard.putData("Auto Chooser 26", autoChooser);
  }

  public static AutoChooser create(final RobotContainer robotContainer) {
    return new AutoChooser(robotContainer);
  }

  // runs in disabled periodic
  public void update() {
    Autos selected = autoChooser.getSelected();

    if (selected == null) {
      System.out.println("NO AUTO WAS CHOSEN, selectedAuto is null!");
      return;
    } else if (selected != lastSelected) {
      System.out.println("REBUILDING AUTO: " + selected.name());

      currentAuto =
          (selected != null)
              ? selected.getStartPoseAndRoutine(factory)
              : Autos.NO_AUTO.getStartPoseAndRoutine(factory);

      lastSelected = selected;
      // preview start pose in AdvantageScope while disabled
      Pose2d currentStartPose = currentAuto.getFirst();
      if (currentStartPose != null) {
        RobotState.getInstance().setAutoStartPose(currentStartPose);
        DrivetrainSubsystem.getInstance().resetPose(currentStartPose); // still red?
      }
    }
  }

  // -------------------------------- HELPERS -------------------------------- //
  public static Pose2d getAllianceAdjustedPose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return ChoreoAllianceFlipUtil.flip(bluePose);
    } else {
      return bluePose;
    }
  }

  public Command getAuto() {
    if (currentAuto == null) {
      return Commands.sequence(
          Commands.print("CURRENTAUTO WAS NULL AT RUNTIME."),
          Commands.print(
              "DRIVERSTATION ALLIANCE PRESENT?: " + DriverStation.getAlliance().isPresent()),
          Commands.none());
    }

    Pose2d currentStartPose = currentAuto.getFirst();
    if (currentStartPose != null) {
      RobotState.getInstance().setAutoStartPose(currentStartPose);
      DrivetrainSubsystem.getInstance().resetPose(currentStartPose);
    }
    return currentAuto.getSecond();
  }
}
