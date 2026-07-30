package com.team2052.lib.vision.questnav;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import lombok.Getter;
import lombok.Setter;

public class QuestNavSubsystem extends SubsystemBase {

  protected final QuestNav questNav;

  private Field2d field2d = new Field2d();

  @Getter private Pose3d lastPose = new Pose3d();
  @Getter private double lastTimestamp = 0.0;
  @Getter @Setter private Transform3d cameraToRobot = new Transform3d();

  // Status
  @Getter private boolean isTracking = false;
  @Getter private boolean isConnected = false;
  @Getter private Time latency = Seconds.of(0);
  @Getter private double batterPercentage = 0.0;

  public QuestNavSubsystem(Transform3d cameraToRobot) {
    this.cameraToRobot = cameraToRobot;
    questNav = new QuestNav();
    SmartDashboard.putData("QuestNav/Quest Pose", field2d);
  }

  public void seedPose(Pose3d pose) {
    lastPose = pose;
    questNav.setPose(pose.transformBy(cameraToRobot));
  }

  public Pose2d getFlatLastPose() {
    return lastPose.toPose2d();
  }

  public void questPeriodic() {
    questNav.commandPeriodic();

    for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
      double timestamp = frame.dataTimestamp();
      if (timestamp > lastTimestamp) {
        lastTimestamp = timestamp;
        lastPose = frame.questPose3d().transformBy(cameraToRobot.inverse());
      }
    }

    if (questNav.isTracking()) {
      isTracking = true;
    } else {
      isTracking = false;
    }

    isConnected = questNav.isConnected();
    latency = Milliseconds.of(questNav.getLatency());
    questNav.getBatteryPercent().ifPresent(battery -> batterPercentage = battery / 100.0);

    SmartDashboard.putBoolean("QuestNav/Is Connected", isConnected);
    SmartDashboard.putBoolean("QuestNav/Is Tracking", isTracking);
    SmartDashboard.putNumber("QuestNav/Latency", latency.in(Milliseconds));
    SmartDashboard.putNumber("QuestNav/Battery", batterPercentage);
    field2d.setRobotPose(getFlatLastPose());
  }
}
