package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(96);

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

    INTAKE_ROLLER_CONSTANTS.slot0kP = 20;
    INTAKE_ROLLER_CONSTANTS.slot0kI = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kD = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kS = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kV = 0;
    INTAKE_ROLLER_CONSTANTS.slot0kA = 0;
    INTAKE_ROLLER_CONSTANTS.maxAngularVelocity = RotationsPerSecond.of(0);
    INTAKE_ROLLER_CONSTANTS.counterClockwisePositive = true;
    INTAKE_ROLLER_CONSTANTS.neutralMode = NeutralModeValue.Coast;
    INTAKE_ROLLER_CONSTANTS.sensorToMechanismRatio = 1; // (30.0 / 12);
    INTAKE_ROLLER_CONSTANTS.statorCurrentLimit = Amps.of(80);
    INTAKE_ROLLER_CONSTANTS.enableStatorCurrentLimit = true;
    INTAKE_ROLLER_CONSTANTS.supplyCurrentLimit = Amps.of(40);
    INTAKE_ROLLER_CONSTANTS.enableSupplyCurrentLimit = true;
  }

  // ----- INTAKE SERVO -----

  public static final Angle UP_POSITION = Rotations.of(0.584697);
  public static final Angle DEPOT_POSITION = Degrees.of(10);
  public static final Angle HALFWAY_POSITION = Degrees.of(37);
  public static final Angle STOW_POSITION = Degrees.of(52.5);
  public static final Angle DOWN_POSITION = Rotations.of(0.01);
  public static final Angle MIN_INTAKE_ARM_ANGLE = Rotations.of(0.01);
  public static final Angle MAX_INTAKE_ARM_ANGLE = Rotations.of(0.590088);

  public static final ServoSubsystemConstants INTAKE_SERVO_CONSTANTS =
      new ServoSubsystemConstants();
  public static final CANCoderConstants INTAKE_ENCODER_CONSTANTS = new CANCoderConstants();

  public static final Angle ENCODER_OFFSET = Rotations.of(-0.0244140625);

  static {
    INTAKE_SERVO_CONSTANTS.name = "Intake Pivot";

    INTAKE_SERVO_CONSTANTS.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.INTAKE_PIVOT_MOTOR.getFirst())
            .withBus(Ports.INTAKE_PIVOT_MOTOR.getSecond());

    INTAKE_SERVO_CONSTANTS.counterClockwisePositive = false;

    INTAKE_SERVO_CONSTANTS.sensorToMechanismRatio = (48.0 / 14);
    INTAKE_SERVO_CONSTANTS.rotorToSensorRatio = (48.0 / 12) * (48.0 / 16);

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
    INTAKE_SERVO_CONSTANTS.acceleration = 3600; // degrees / second / second

    INTAKE_SERVO_CONSTANTS.softwareMax = MAX_INTAKE_ARM_ANGLE;
    INTAKE_SERVO_CONSTANTS.softwareMin = Rotations.of(0);
    INTAKE_SERVO_CONSTANTS.enableSupplyCurrentLimit = true;
    INTAKE_SERVO_CONSTANTS.supplyCurrentLimit = Amps.of(20);

    INTAKE_SERVO_CONSTANTS.maxOutput = Volts.of(12.0);
    INTAKE_SERVO_CONSTANTS.neutralMode = NeutralModeValue.Coast;
    INTAKE_SERVO_CONSTANTS.slot0kDeadband = (1 / 120.0);

    INTAKE_SERVO_CONSTANTS.sensorMode = FeedbackSensorSourceValue.FusedCANcoder;

    INTAKE_ENCODER_CONSTANTS.id = Ports.INTAKE_ENCODER;
    INTAKE_ENCODER_CONSTANTS.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    INTAKE_ENCODER_CONSTANTS.config.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    INTAKE_ENCODER_CONSTANTS.config.MagnetSensor.MagnetOffset = ENCODER_OFFSET.in(Rotations);
    INTAKE_ENCODER_CONSTANTS.statusSignalUpdateFrequency = Hertz.of(50);
  }
}
