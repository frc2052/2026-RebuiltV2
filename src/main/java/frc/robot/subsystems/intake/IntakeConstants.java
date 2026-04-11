package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.RollerSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Ports;

public final class IntakeConstants {

  // ----- INTAKE ROLLER -----
  public static final RollerSubsystemConstants INTAKE_ROLLER_CONSTANTS =
      new RollerSubsystemConstants();
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(160);

  static {
    INTAKE_ROLLER_CONSTANTS.name = "Intake Roller";
    INTAKE_ROLLER_CONSTANTS.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.LEFT_INTAKE_MOTOR.getFirst())
            .withBus(Ports.LEFT_INTAKE_MOTOR.getSecond());
    INTAKE_ROLLER_CONSTANTS.followerTalonFXConstants =
        new TalonFXConstants[] {
          new TalonFXConstants()
              .withId(Ports.RIGHT_INTAKE_MOTOR.getFirst())
              .withBus(Ports.RIGHT_INTAKE_MOTOR.getSecond())
              .withInvertMotorOutput(MotorAlignmentValue.Opposed)
        };

    INTAKE_ROLLER_CONSTANTS.slot0kP = 4;
    INTAKE_ROLLER_CONSTANTS.slot0kI = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kD = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kS = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kV = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kA = 0;
    INTAKE_ROLLER_CONSTANTS.maxAngularVelocity = RotationsPerSecond.of(0);
    INTAKE_ROLLER_CONSTANTS.counterClockwisePositive = false;
    INTAKE_ROLLER_CONSTANTS.neutralMode = NeutralModeValue.Coast;
    INTAKE_ROLLER_CONSTANTS.sensorToMechanismRatio = 12.0 / 30;
    INTAKE_ROLLER_CONSTANTS.statorCurrentLimit = Amps.of(60);
    INTAKE_ROLLER_CONSTANTS.enableStatorCurrentLimit = true;
    INTAKE_ROLLER_CONSTANTS.supplyCurrentLimit = Amps.of(40);
    INTAKE_ROLLER_CONSTANTS.enableSupplyCurrentLimit = true;
  }

  // ----- INTAKE SERVO -----

  public static final Angle IN_POSITION = Rotations.of(0.12);
  public static final Angle HALFWAY_POSITION = Degrees.of(32);
  public static final Angle STOW_POSITION = Degrees.of(52.5);
  public static final Angle OUT_POSITION = Rotations.of(0);
  public static final Angle MIN_INTAKE_ARM_ANGLE = Rotations.of(0);
  public static final Angle MAX_INTAKE_ARM_ANGLE = Rotations.of(0.177);

  public static final ServoSubsystemConstants INTAKE_SERVO_CONSTANTS =
      new ServoSubsystemConstants();
  public static final CANCoderConstants INTAKE_ENCODER_CONSTANTS = new CANCoderConstants();

  public static final Angle ENCODER_OFFSET = Rotations.of(0.336425);

  static {
    INTAKE_SERVO_CONSTANTS.name = "Intake Pivot";

    INTAKE_SERVO_CONSTANTS.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.INTAKE_PIVOT_MOTOR.getFirst())
            .withBus(Ports.INTAKE_PIVOT_MOTOR.getSecond());

    INTAKE_SERVO_CONSTANTS.counterClockwisePositive = true;

    INTAKE_SERVO_CONSTANTS.sensorToMechanismRatio = 1;
    // 1 because we use RemoteCANCoder instead of the rotor encoder
    // (42.0 / 12.0) * (42.0 / 16.0) * (48.0 / 12.0);

    INTAKE_SERVO_CONSTANTS.slot0kP = 80;
    INTAKE_SERVO_CONSTANTS.slot0kI = 0;
    INTAKE_SERVO_CONSTANTS.slot0kD = 0.1;
    INTAKE_SERVO_CONSTANTS.slot0kS = 1;
    INTAKE_SERVO_CONSTANTS.slot0kV = 0;
    INTAKE_SERVO_CONSTANTS.slot0kA = 0;
    INTAKE_SERVO_CONSTANTS.slot0kG = 0;

    INTAKE_SERVO_CONSTANTS.slot1kP = 0.0;
    INTAKE_SERVO_CONSTANTS.slot1kI = 0.0;
    INTAKE_SERVO_CONSTANTS.slot1kD = 0.0;

    INTAKE_SERVO_CONSTANTS.cruiseVelocity = 360; // degrees / second
    INTAKE_SERVO_CONSTANTS.acceleration = 360; // degrees / second / second

    INTAKE_SERVO_CONSTANTS.softwareMax = Rotations.of(0.175);
    INTAKE_SERVO_CONSTANTS.softwareMin = Rotations.of(0);
    INTAKE_SERVO_CONSTANTS.enableSupplyCurrentLimit = true;
    INTAKE_SERVO_CONSTANTS.supplyCurrentLimit = Amps.of(10);

    INTAKE_SERVO_CONSTANTS.maxOutput = Volts.of(12.0);
    INTAKE_SERVO_CONSTANTS.neutralMode = NeutralModeValue.Brake;

    INTAKE_ENCODER_CONSTANTS.id = Ports.INTAKE_PIVOT_ENCODER;
  }
}
