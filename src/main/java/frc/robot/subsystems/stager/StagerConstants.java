package frc.robot.subsystems.stager;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.RollerSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;
import frc.robot.util.Ports;

public class StagerConstants {
  public static final RollerSubsystemConstants ROLLER_CONFIG = new RollerSubsystemConstants();

  static {
    ROLLER_CONFIG.name = "Stager";
    ROLLER_CONFIG.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.STAGER_TOP_MOTOR.getFirst())
            .withBus(Ports.STAGER_TOP_MOTOR.getSecond());
    ROLLER_CONFIG.followerTalonFXConstants =
        new TalonFXConstants[] {
          new TalonFXConstants()
              .withId(Ports.STAGER_BOTTOM_MOTOR.getFirst())
              .withBus(Ports.STAGER_BOTTOM_MOTOR.getSecond())
              .withInvertMotorOutput(MotorAlignmentValue.Aligned)
        };

    ROLLER_CONFIG.slot0kP = 1;
    ROLLER_CONFIG.slot0kI = 0;
    ROLLER_CONFIG.slot0kD = 0;
    ROLLER_CONFIG.slot0kS = 0;
    ROLLER_CONFIG.slot0kV = 0;
    ROLLER_CONFIG.slot0kA = 0;
    ROLLER_CONFIG.counterClockwisePositive = true;
    ROLLER_CONFIG.neutralMode = NeutralModeValue.Coast;
    ROLLER_CONFIG.sensorToMechanismRatio = 1.0;

    ROLLER_CONFIG.statorCurrentLimit = Amps.of(80);
    ROLLER_CONFIG.enableStatorCurrentLimit = false;
    ROLLER_CONFIG.supplyCurrentLimit = Amps.of(40);
    ROLLER_CONFIG.enableSupplyCurrentLimit = false;
  }
}
