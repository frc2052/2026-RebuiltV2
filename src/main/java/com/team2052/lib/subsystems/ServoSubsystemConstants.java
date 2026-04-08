package com.team2052.lib.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class ServoSubsystemConstants {
  public String name = "UNNAMED_MOTOR";
  public TalonFXConstants leaderTalonFXConstants = new TalonFXConstants();
  public TalonFXConstants[] followerTalonFXConstants = new TalonFXConstants[0];

  public double loopTime = Constants.MAIN_LOOP_PERIOD.in(Seconds);
  public Time timeout = Milliseconds.of(100);

  public NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public double homePosition = 0.0;

  /**
   * This is the ratio of sensor rotations to the mechanism's output. This is equivalent to the
   * mechanism's gear ratio if the sensor is located on the input of a gearbox (motor shaft). If
   * sensor is on the output of a gearbox, then this is typically set to 1.
   */
  public double sensorToMechanismRatio = 1.0;

  public double rotorToSensorRatio = 1.0;

  public final int motionMagicSlot = 0;
  public final int positionSlot = 1;

  // Slot 0 should be used when using motion magic
  public double slot0kP = 0;
  public double slot0kI = 0;
  public double slot0kD = 0;
  public double slot0kV = 0;
  public double slot0kA = 0;
  public double slot0kS = 0;
  public double slot0kG = 0;
  public double slot0kDeadband = 0;

  // Slot 1 should be used for position control
  public double slot1kP = 0;
  public double slot1kI = 0;
  public double slot1kD = 0;
  public double slot1kDeadband = 0;

  // Profiling configs, used for things like Motion Magic or Trapezoidal Profile
  public double velocityFeedforward = 0;
  public double arbitraryFeedforward = 0;
  public double cruiseVelocity = 0;
  public double acceleration = 0;
  public double jerk = 0;
  public double rampRate = 0.0;

  public Current supplyCurrentLimit = Amps.of(60);
  public Current supplyCurrentThreshold = Amps.of(60);
  public Time supplyCurrentTimeout = Seconds.of(0.0);
  public boolean enableSupplyCurrentLimit = false;

  public Current statorCurrentLimit = Amps.of(40);
  public boolean enableStatorCurrentLimit = false;

  public Voltage maxOutput = Volts.of(12.0);
  public Angle softwareMax = Degrees.of(Double.POSITIVE_INFINITY);
  public Angle softwareMin = Degrees.of(Double.NEGATIVE_INFINITY);
  public boolean reverseSoftLimitEnable = true;
  public boolean forwardSoftLimitEnable = true;

  public boolean counterClockwisePositive = false;

  /**
   * Sets the proportional gain for the motor controller.
   *
   * <p>The units for this gain is dependent on the control mode. Since this gain is multiplied by
   * error in the input, the units should be defined as units of output per unit of input error.
   *
   * @param kP the proportional gain
   * @return this
   */
  public ServoSubsystemConstants withKp(double kP) {
    this.slot0kP = kP;
    return this;
  }

  /**
   * Sets the integral gain for the motor controller.
   *
   * <p>The units for this gain is dependent on the control mode. Since this gain is multiplied by
   * error in the input integrated over time (in units of seconds), the units should be defined as
   * units of output per unit of integrated input error. For example, when controlling velocity
   * using a duty cycle closed loop, integrating velocity over time results in rps * s = rotations.
   * Therefore, the units for the integral gain will be duty cycle per rotation of accumulated
   * error, or 1/rot.
   *
   * @param kI the integral gain
   * @return this
   */
  public ServoSubsystemConstants withKi(double kI) {
    this.slot0kI = kI;
    return this;
  }

  /**
   * Sets the derivative gain for the motor controller.
   *
   * <p>The units for this gain is dependent on the control mode. Since this gain is multiplied by
   * the rate of change of error in the input (in units of per second), the units should be defined
   * as units of output per unit of input error rate. For example, when controlling velocity using a
   * duty cycle closed loop, the units for the derivative gain will be duty cycle per (rps / s), or
   * s / rot.
   *
   * @param kD the derivative gain
   * @return this
   */
  public ServoSubsystemConstants withKd(double kD) {
    this.slot0kD = kD;
    return this;
  }

  /**
   * Sets the velocity gain for the motor controller.
   *
   * @param kV the velocity gain
   * @return this
   */
  public ServoSubsystemConstants withKv(double kV) {
    this.slot0kV = kV;
    return this;
  }

  /**
   * Sets the acceleration gain for the motor controller.
   *
   * @param kA the acceleration gain
   * @return this
   */
  public ServoSubsystemConstants withKa(double kA) {
    this.slot0kA = kA;
    return this;
  }

  /**
   * Sets the static gain for the motor controller.
   *
   * @param kS the static gain
   * @return this
   */
  public ServoSubsystemConstants withKs(double kS) {
    this.slot0kS = kS;
    return this;
  }

  /**
   * Sets the gravity gain for the motor controller.
   *
   * @param kG the gravity gain
   * @return this
   */
  public ServoSubsystemConstants withKg(double kG) {
    this.slot0kG = kG;
    return this;
  }

  /**
   * Sets the deadband for the motor controller.
   *
   * @param deadband the deadband
   * @return this
   */
  public ServoSubsystemConstants withDeadband(int deadband) {
    this.slot0kDeadband = deadband;
    return this;
  }

  /**
   * Sets the proportional gain for the position PID controller.
   *
   * @param positionKp the proportional gain for the position PID controller
   * @return this
   */
  public ServoSubsystemConstants withPositionKp(double positionKp) {
    this.slot1kP = positionKp;
    return this;
  }

  /**
   * Sets the integral gain for the position PID controller.
   *
   * @param positionKi the integral gain for the position PID controller
   * @return this
   */
  public ServoSubsystemConstants withPositionKi(double positionKi) {
    this.slot1kI = positionKi;
    return this;
  }

  /**
   * Sets the derivative gain for the position PID controller.
   *
   * @param positionKd the derivative gain for the position PID controller
   * @return this
   */
  public ServoSubsystemConstants withPositionKd(double positionKd) {
    this.slot1kD = positionKd;
    return this;
  }

  public ServoSubsystemConstants withPositionDeadband(double positionDeadband) {
    this.slot1kDeadband = positionDeadband;
    return this;
  }

  /**
   * Sets the cruise velocity for the motor controller.
   *
   * @param cruiseVelocity the cruise velocity
   * @return this
   */
  public ServoSubsystemConstants withCruiseVelocity(double cruiseVelocity) {
    this.cruiseVelocity = cruiseVelocity;
    return this;
  }

  /**
   * Sets the acceleration for the motor controller.
   *
   * @param acceleration the acceleration
   * @return this
   */
  public ServoSubsystemConstants withAcceleration(double acceleration) {
    this.acceleration = acceleration;
    return this;
  }

  /**
   * Sets the jerk for the motor controller.
   *
   * @param jerk the jerk
   * @return this
   */
  public ServoSubsystemConstants withJerk(double jerk) {
    this.jerk = jerk;
    return this;
  }

  /**
   * Sets the ramp rate for the motor controller.
   *
   * @param rampRate the ramp rate
   * @return this
   */
  public ServoSubsystemConstants withRampRate(double rampRate) {
    this.rampRate = rampRate;
    return this;
  }

  /**
   * Sets the maximum voltage for the motor controller.
   *
   * @param maxVoltage the maximum voltage
   * @return this
   */
  public ServoSubsystemConstants withMaxVoltage(Voltage maxVoltage) {
    this.maxOutput = maxVoltage;
    return this;
  }

  /**
   * The amount of supply current allowed. This is only applicable for non-torque current control
   * modes.
   *
   * @param supplyCurrentLimit the supply current limit
   * @return this
   */
  public ServoSubsystemConstants withSupplyCurrentLimit(Current supplyCurrentLimit) {
    this.supplyCurrentLimit = supplyCurrentLimit;
    return this;
  }

  /**
   * Enable the supply current limit. This is only applicable for non-torque current control modes.
   *
   * @param enableSupplyCurrentLimit enable the supply current limit
   * @return this
   */
  public ServoSubsystemConstants withEnableSupplyCurrentLimit(boolean enableSupplyCurrentLimit) {
    this.enableSupplyCurrentLimit = enableSupplyCurrentLimit;
    return this;
  }

  /**
   * The amount of current allowed in the motor (motoring and regen current). This is only
   * applicable for
   *
   * @param statorCurrentLimit the stator current limit
   * @return this
   */
  public ServoSubsystemConstants withStatorCurrentLimit(Current statorCurrentLimit) {
    this.statorCurrentLimit = statorCurrentLimit;
    return this;
  }

  /**
   * Enable the stator current limit. This is only applicable for non-torque current control modes.
   *
   * @param enableStatorCurrentLimit enable the stator current limit
   * @return this
   */
  public ServoSubsystemConstants withEnableStatorCurrentLimit(boolean enableStatorCurrentLimit) {
    this.enableStatorCurrentLimit = enableStatorCurrentLimit;
    return this;
  }

  /**
   * Sets the maximum units limit for the motor controller.
   *
   * @param softwareMax the maximum units limit
   * @return this
   */
  public ServoSubsystemConstants withMaxUnitsLimit(Angle softwareMax) {
    this.softwareMax = softwareMax;
    return this;
  }

  /**
   * Sets the minimum units limit for the motor controller.
   *
   * @param softwareMin the minimum units limit
   * @return this
   */
  public ServoSubsystemConstants withMinUnitsLimit(Angle softwareMin) {
    this.softwareMin = softwareMin;
    return this;
  }

  /**
   * Sets the ratio of sensor rotations to the mechanism's output.
   *
   * @param sensorToMechanismRatio the ratio of sensor rotations to the mechanism's output
   * @return this
   */
  public ServoSubsystemConstants withSensorToMechanismRatio(double sensorToMechanismRatio) {
    this.sensorToMechanismRatio = sensorToMechanismRatio;
    return this;
  }

  /** Holds id, bus, and motor alignment value for a TalonFX motor controller */
  public static class TalonFXConstants {
    public int id = -1;
    public CANBus bus = new CANBus();
    public MotorAlignmentValue alignmentValue = MotorAlignmentValue.Opposed;

    public TalonFXConstants withId(int id) {
      this.id = id;
      return this;
    }

    public TalonFXConstants withBus(CANBus bus) {
      this.bus = bus;
      return this;
    }

    public TalonFXConstants withInvertMotorOutput(MotorAlignmentValue alignmentValue) {
      this.alignmentValue = alignmentValue;
      return this;
    }
  }
}
