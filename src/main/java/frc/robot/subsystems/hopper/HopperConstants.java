package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants.TalonFXConstants;

import edu.wpi.first.units.measure.*;
import frc.robot.util.Ports;

public final class HopperConstants {

    public static final ServoSubsystemConstants SERVO_CONSTANTS = new ServoSubsystemConstants();

    public static final Angle EXTENDED_POSITION = Rotation.of(10);
    public static final Angle RETRACTED_POSITION = Rotation.of(0);

    public static final Angle MIN_HOPPER_POSITION = Rotation.of(0);
    public static final Angle MAX_HOPPER_POSITION = Rotation.of(10);

    public static final double OPEN_LOOP_HOMING_SPEED = -0.01;

    public static final double OPEN_LOOP_FALLING_SPEED = -0.2;

    public static final Angle DISTANCE_TO_BOTTOM_THRESHOLD = Rotation.of(0);

    static {
        SERVO_CONSTANTS.name = "Hopper Extender";
        
        SERVO_CONSTANTS.leaderTalonFXConstants = new TalonFXConstants()
            .withId(Ports.HOPPER_EXTENDER_MOTOR.getFirst())
            .withBus(Ports.HOPPER_EXTENDER_MOTOR.getSecond());
        SERVO_CONSTANTS.counterClockwisePositive = true;

        SERVO_CONSTANTS.slot1kP = 0;
        SERVO_CONSTANTS.slot1kD = 0;
        SERVO_CONSTANTS.cruiseVelocity = 0;
        SERVO_CONSTANTS.acceleration = 0;

        SERVO_CONSTANTS.softwareMin = MIN_HOPPER_POSITION;
        SERVO_CONSTANTS.softwareMax = MAX_HOPPER_POSITION;

        SERVO_CONSTANTS.maxOutput = Volts.of(12.0);
        SERVO_CONSTANTS.neutralMode = NeutralModeValue.Brake;
    }
    
}
