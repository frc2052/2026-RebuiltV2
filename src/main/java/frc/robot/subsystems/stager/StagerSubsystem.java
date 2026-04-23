package frc.robot.subsystems.stager;

import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StagerSubsystem extends RollerSubsystem {
  private static StagerSubsystem INSTANCE;

  public static StagerSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new StagerSubsystem();
    }
    return INSTANCE;
  }

  public StagerSubsystem() {
    super(StagerConstants.ROLLER_CONFIG);
  }

  public void runAtFiringVelocity() {
    setGoalVelocityTorque(StagerConstants.FIRING_VELOCITY);
  }

  public Command scoreCommand() {
    return runAtVelocityCommand(StagerConstants.FIRING_VELOCITY);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
