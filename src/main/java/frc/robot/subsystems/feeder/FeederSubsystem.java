package frc.robot.subsystems.feeder;

import com.team2052.lib.subsystems.RollerSubsystem;

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

  @Override
  public void periodic() {
    super.periodic();
  }
}
