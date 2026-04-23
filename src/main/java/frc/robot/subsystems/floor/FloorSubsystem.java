package frc.robot.subsystems.floor;

import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class FloorSubsystem extends RollerSubsystem {
  private static FloorSubsystem INSTANCE;

  public static FloorSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new FloorSubsystem();
    }
    return INSTANCE;
  }

  public FloorSubsystem() {
    super(FloorConstants.ROLLER_CONFIG);
  }

  public void runAtFiringVelocity() {
    setGoalVelocityTorque(FloorConstants.FIRING_VELOCITY);
  }

  public Command scoreCommand() {
    return runAtVelocityCommand(FloorConstants.FIRING_VELOCITY);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
