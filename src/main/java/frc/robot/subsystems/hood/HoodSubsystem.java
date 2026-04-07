package frc.robot.subsystems.hood;

import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemWithCANCoder;

public class HoodSubsystem extends ServoSubsystemWithCANCoder{
    private HoodSubsystem(ServoSubsystemConstants constants, CANCoderConstants canCoderConstants) {
    super(constants, canCoderConstants);
  }

  private static HoodSubsystem INSTANCE;

  public static HoodSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new HoodSubsystem(HoodConstants.SERVO_CONSTANTS, HoodConstants.ENCODER_CONSTANTS);
    }
    return INSTANCE;
  }
}
