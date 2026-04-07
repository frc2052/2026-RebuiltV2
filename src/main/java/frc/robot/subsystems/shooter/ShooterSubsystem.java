package frc.robot.subsystems.shooter;

import com.team2052.lib.subsystems.RollerSubsystem;

public class ShooterSubsystem extends RollerSubsystem {
  private static ShooterSubsystem INSTANCE;

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  }

  public ShooterSubsystem() {
    super(ShooterConstants.ROLLER_CONFIG);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
