package frc.robot.subsystems.floor;

import com.team2052.lib.subsystems.RollerSubsystem;

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

  @Override
  public void periodic() {
    super.periodic();
  }
}
