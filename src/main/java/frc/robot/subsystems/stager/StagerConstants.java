package frc.robot.subsystems.stager;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.RollerSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Ports;

public class StagerConstants {
  public static final RollerSubsystemConstants ROLLER_CONFIG = new RollerSubsystemConstants();

  public static final AngularVelocity FIRING_VELOCITY = RotationsPerSecond.of(40);

  static {
    ROLLER_CONFIG.name = "Stager";
    ROLLER_CONFIG.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.STAGER_TOP_MOTOR.getFirst())
            .withBus(Ports.STAGER_TOP_MOTOR.getSecond());

    ROLLER_CONFIG.slot0kP = 10;
    ROLLER_CONFIG.slot0kI = 0;
    ROLLER_CONFIG.slot0kD = 0;
    ROLLER_CONFIG.slot0kS = 0;
    ROLLER_CONFIG.slot0kV = 0;
    ROLLER_CONFIG.slot0kA = 0;

    ROLLER_CONFIG.followerRateHz = 1000;

    ROLLER_CONFIG.maxAngularVelocity = RotationsPerSecond.of(96);
    ROLLER_CONFIG.counterClockwisePositive = true;
    ROLLER_CONFIG.neutralMode = NeutralModeValue.Coast;
    ROLLER_CONFIG.slot0kDeadband = 0.05;
    ROLLER_CONFIG.sensorToMechanismRatio = 1;
    ROLLER_CONFIG.statorCurrentLimit = Amps.of(40);
    ROLLER_CONFIG.enableStatorCurrentLimit = true;
  }
}
