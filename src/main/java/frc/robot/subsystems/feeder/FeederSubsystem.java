package frc.robot.subsystems.feeder;

import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class FeederSubsystem extends RollerSubsystem {
  private static FeederSubsystem INSTANCE;

  public static FeederSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new FeederSubsystem();
    }
    return INSTANCE;
  }

  public FeederSubsystem() {
    super(FeederConstants.ROLLER_CONFIG);
  }

  public void runAtFiringVelocity() {
    setGoalVelocity(FeederConstants.FIRING_VELOCITY);
  }

  public Command scoreCommand() {
    return runAtVelocityCommand(FeederConstants.FIRING_VELOCITY);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
