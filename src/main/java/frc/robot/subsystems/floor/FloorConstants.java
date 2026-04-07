package frc.robot.subsystems.floor;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.RollerSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;

import frc.robot.util.Ports;

public final class FloorConstants {
    public static final RollerSubsystemConstants ROLLER_CONFIG = new RollerSubsystemConstants();
    static {
      ROLLER_CONFIG.name = "Floor";
      ROLLER_CONFIG.leaderTalonFXConstants =
          new TalonFXConstants()
              .withId(Ports.FLOOR_LEFT_MOTOR.getFirst())
              .withBus(Ports.FLOOR_LEFT_MOTOR.getSecond());
      ROLLER_CONFIG.followerTalonFXConstants =
          new TalonFXConstants[] {
            new TalonFXConstants()
                .withId(Ports.FLOOR_RIGHT_MOTOR.getFirst())
                .withBus(Ports.FLOOR_RIGHT_MOTOR.getSecond())
                .withInvertMotorOutput(MotorAlignmentValue.Opposed)
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
