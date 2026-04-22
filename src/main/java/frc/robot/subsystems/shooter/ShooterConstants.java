package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.RollerSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Ports;

public final class ShooterConstants {
  public static final RollerSubsystemConstants ROLLER_CONFIG = new RollerSubsystemConstants();
  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(40);

  public static final double BANG_BANG_SPEED = 1;

  public static final AngularVelocity PID_USE_TOLERANCE = RotationsPerSecond.of(10);

  static {
    ROLLER_CONFIG.name = "Shooter";
    ROLLER_CONFIG.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.SHOOTER_BOTTOM_LEFT.getFirst())
            .withBus(Ports.SHOOTER_BOTTOM_LEFT.getSecond());
    ROLLER_CONFIG.followerTalonFXConstants =
        new TalonFXConstants[] {
          new TalonFXConstants()
              .withId(Ports.SHOOTER_TOP_LEFT.getFirst())
              .withBus(Ports.SHOOTER_TOP_LEFT.getSecond())
              .withInvertMotorOutput(MotorAlignmentValue.Aligned),
          new TalonFXConstants()
              .withId(Ports.SHOOTER_BOTTOM_RIGHT.getFirst())
              .withBus(Ports.SHOOTER_BOTTOM_RIGHT.getSecond())
              .withInvertMotorOutput(MotorAlignmentValue.Opposed),
          new TalonFXConstants()
              .withId(Ports.SHOOTER_TOP_RIGHT.getFirst())
              .withBus(Ports.SHOOTER_TOP_RIGHT.getSecond())
              .withInvertMotorOutput(MotorAlignmentValue.Opposed)
        };

    ROLLER_CONFIG.slot0kP = 7.5; // 9
    ROLLER_CONFIG.slot0kI = 5;
    ROLLER_CONFIG.slot0kD = 0.1; // 0.12
    ROLLER_CONFIG.slot0kS = 4.1;
    ROLLER_CONFIG.slot0kV = 0.16; // 0.16
    ROLLER_CONFIG.slot0kA = 0;

    ROLLER_CONFIG.followerRateHz = 1000;

    ROLLER_CONFIG.maxAngularVelocity = RotationsPerSecond.of(96);
    ROLLER_CONFIG.counterClockwisePositive = false;
    ROLLER_CONFIG.neutralMode = NeutralModeValue.Coast;
    ROLLER_CONFIG.slot0kDeadband = 0.5;
    ROLLER_CONFIG.sensorToMechanismRatio = 1;
    ROLLER_CONFIG.statorCurrentLimit = Amps.of(80);
    ROLLER_CONFIG.enableStatorCurrentLimit = true;
    ROLLER_CONFIG.supplyCurrentLimit = Amps.of(40);
    ROLLER_CONFIG.enableSupplyCurrentLimit = true;
  }
}
