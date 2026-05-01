package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.Ports;

/*
 *  6 deg to 51 deg physically
 */
public class HoodConstants {
  public static final Angle HOOD_MIN_ANGLE = Rotations.of(0.01);
  public static final Angle HOOD_MAX_ANGLE = Rotations.of(0.11792);
  public static final double OPEN_LOOP_HOMING_SPEED = -0.1;
  // before new gear 0.728281
  // after new gear 0.736816
  public static final Angle ENCODER_OFFSET = Rotations.of(-0.728281);

  public static final ServoSubsystemConstants SERVO_CONSTANTS = new ServoSubsystemConstants();

  public static final double SOTM_RADIAL_VELOCITY_COMPENSATION = 1.2;

  static {
    SERVO_CONSTANTS.name = "Hood";

    SERVO_CONSTANTS.leaderTalonFXConstants =
        new TalonFXConstants()
            .withId(Ports.HOOD_MOTOR.getFirst())
            .withBus(Ports.HOOD_MOTOR.getSecond());
    SERVO_CONSTANTS.counterClockwisePositive = false;

    SERVO_CONSTANTS.slot0kP = 200;
    SERVO_CONSTANTS.slot0kI = 0;
    SERVO_CONSTANTS.slot0kD = 0;
    SERVO_CONSTANTS.slot0kS = 0;
    SERVO_CONSTANTS.slot0kV = 0;
    SERVO_CONSTANTS.slot0kA = 0;
    SERVO_CONSTANTS.slot0kG = 0;

    SERVO_CONSTANTS.slot1kP = 0.0;
    SERVO_CONSTANTS.slot1kI = 0.0;
    SERVO_CONSTANTS.slot1kD = 0.0;

    SERVO_CONSTANTS.cruiseVelocity = 720; // degrees / second
    SERVO_CONSTANTS.acceleration = 10800; // degrees / second / second

    SERVO_CONSTANTS.sensorToMechanismRatio = (60.0 / 34.0);
    SERVO_CONSTANTS.statorCurrentLimit = Amps.of(60);
    SERVO_CONSTANTS.enableStatorCurrentLimit = false;
    SERVO_CONSTANTS.supplyCurrentLimit = Amps.of(30);
    SERVO_CONSTANTS.enableSupplyCurrentLimit = false;

    SERVO_CONSTANTS.softwareMin = HOOD_MIN_ANGLE;
    SERVO_CONSTANTS.softwareMax = HOOD_MAX_ANGLE;

    SERVO_CONSTANTS.maxOutput = Volts.of(12.0);
    SERVO_CONSTANTS.neutralMode = NeutralModeValue.Brake;

    SERVO_CONSTANTS.sensorMode = FeedbackSensorSourceValue.RemoteCANcoder;
  }

  public static final CANCoderConstants ENCODER_CONSTANTS = new CANCoderConstants();

  static {
    ENCODER_CONSTANTS.id = Ports.HOOD_ENCODER;
    ENCODER_CONSTANTS.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    ENCODER_CONSTANTS.config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    ENCODER_CONSTANTS.config.MagnetSensor.MagnetOffset = ENCODER_OFFSET.in(Rotations);
  }
}
