package frc.robot.subsystems.stager;

import com.team2052.lib.subsystems.RollerSubsystem;

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

    @Override
    public void periodic() {
        super.periodic();
    }
}
