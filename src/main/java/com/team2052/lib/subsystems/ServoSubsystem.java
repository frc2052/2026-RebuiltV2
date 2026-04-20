package com.team2052.lib.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoSubsystem extends SubsystemBase {
  protected final TalonFX leader;
  protected final TalonFX[] followers;
  protected TalonFXConfiguration leaderConfig;
  protected TalonFXConfiguration[] followerConfigs;
  protected ServoSubsystemConstants constants;
  protected ControlMode mode = ControlMode.OPEN_LOOP;
  protected double demand; // demand is either: open loop percent OR closed loop degrees

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final BaseStatusSignal[] motorSignals;

  public ServoSubsystem(ServoSubsystemConstants constants) {
    super(constants.name);
    this.constants = constants;

    leader = new TalonFX(constants.leaderTalonFXConstants.id, constants.leaderTalonFXConstants.bus);
    followers = new TalonFX[constants.followerTalonFXConstants.length];

    leaderConfig = new TalonFXConfiguration();
    followerConfigs = new TalonFXConfiguration[constants.followerTalonFXConstants.length];

    leaderConfig.Feedback.FeedbackSensorSource = constants.sensorMode;
    leaderConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechanismRatio;
    leaderConfig.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio;

    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        constants.softwareMax.in(Rotations);
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants.forwardSoftLimitEnable;
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        constants.softwareMin.in(Rotations);
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants.reverseSoftLimitEnable;

    leaderConfig.Slot0.kP = constants.slot0kP;
    leaderConfig.Slot0.kI = constants.slot0kI;
    leaderConfig.Slot0.kD = constants.slot0kD;
    leaderConfig.Slot0.kV = constants.slot0kV;
    leaderConfig.Slot0.kA = constants.slot0kA;
    leaderConfig.Slot0.kS = constants.slot0kS;

    leaderConfig.Slot1.kP = constants.slot1kP;
    leaderConfig.Slot1.kI = constants.slot1kI;
    leaderConfig.Slot1.kD = constants.slot1kD;

    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity;
    leaderConfig.MotionMagic.MotionMagicAcceleration = constants.acceleration;
    leaderConfig.MotionMagic.MotionMagicJerk = constants.jerk;

    leaderConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constants.rampRate;
    leaderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = constants.rampRate;
    leaderConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = constants.rampRate;

    leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = constants.rampRate;
    leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = constants.rampRate;
    leaderConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = constants.rampRate;

    leaderConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit.in(Amps);
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.enableSupplyCurrentLimit;
    leaderConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit.in(Amps);
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = constants.enableStatorCurrentLimit;

    leaderConfig.Voltage.PeakForwardVoltage = constants.maxOutput.in(Volts);
    leaderConfig.Voltage.PeakReverseVoltage = -constants.maxOutput.in(Volts);

    leaderConfig.MotorOutput.PeakForwardDutyCycle = constants.maxOutput.in(Volts) / 12.0;
    leaderConfig.MotorOutput.PeakReverseDutyCycle = -constants.maxOutput.in(Volts) / 12.0;
    leaderConfig.MotorOutput.Inverted =
        (constants.counterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    leaderConfig.MotorOutput.NeutralMode = constants.neutralMode; // Brake or Coast

    // Configure followers
    for (int i = 0; i < followers.length; ++i) {
      int lID = constants.leaderTalonFXConstants.id; // Leader ID
      int fID = constants.followerTalonFXConstants[i].id; // Follower ID
      CANBus fBus = constants.followerTalonFXConstants[i].bus; // Follower CAN bus
      MotorAlignmentValue fAligned =
          constants
              .followerTalonFXConstants[i]
              .alignmentValue; // If the motors rotate in the same direction or not

      if (!fBus.equals(constants.leaderTalonFXConstants.bus)) {
        throw new RuntimeException("Leader and Follower TalonFXs must be on the same CAN bus.");
      }

      try (TalonFX followerMotor = new TalonFX(fID, fBus)) {
        followerMotor.clearStickyFaults();
        followerMotor.setControl(new Follower(lID, fAligned));

        followers[i] = followerMotor;
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerMotor.getConfigurator().refresh(followerConfig);
        followerConfig.MotorOutput.NeutralMode = constants.neutralMode;
        followerMotor.getConfigurator().apply(followerConfig);
        followerConfigs[i] = followerConfig;
        followerMotor.optimizeBusUtilization();
      }
    }

    leader.getConfigurator().apply(leaderConfig, constants.timeout.in(Seconds));
    demand = 0.0;

    leader.optimizeBusUtilization();

    positionSignal = leader.getPosition();
    velocitySignal = leader.getVelocity();

    motorSignals = new BaseStatusSignal[] {positionSignal, velocitySignal};

    BaseStatusSignal.setUpdateFrequencyForAll(20.0, motorSignals);
  }

  /**
   * Get the current position of the leader motor
   *
   * @return Angle
   */
  public Angle getPosition() {
    return positionSignal.getValue();
  }

  /**
   * Get the current velocity of the leader motor
   *
   * @return AngularVelocity
   */
  public AngularVelocity getVelocity() {
    return velocitySignal.getValue();
  }

  /**
   * Get the current setpoint of the leader motor if in closed loop control
   *
   * @return degrees or NaN if in open loop
   */
  public double getSetpoint() {
    return (mode == ControlMode.MOTION_MAGIC || mode == ControlMode.POSITION_PID)
        ? demand
        : Double.NaN;
  }

  /** Set the leader motor's encoder position to zero */
  protected void zeroMotor() {
    leader.setPosition(0);
  }

  /**
   * Set the leader motor's encoder position to a specific angle
   *
   * @param angle The angle to set the encoder to
   */
  protected void resetMotorEncoderTo(Angle angle) {
    leader.setPosition(angle.in(Rotations));
  }

  /**
   * Set the leader motor to open loop control with a specific demand
   *
   * @param demand The demand value for open loop control (usually percent output)
   */
  protected void setOpenLoop(double demand) {
    this.demand = demand;
    if (mode != ControlMode.OPEN_LOOP) {
      mode = ControlMode.OPEN_LOOP;
    }
  }

  /**
   * Set the leader motor to Motion Magic control with a specific setpoint
   *
   * @param demand The setpoint angle for Motion Magic control
   */
  protected void setSetpointMotionMagic(Angle demand) {
    this.demand = demand.in(Degrees);
    if (mode != ControlMode.MOTION_MAGIC) {
      mode = ControlMode.MOTION_MAGIC;
    }
  }

  /**
   * Set the leader motor to Position PID control with a specific setpoint
   *
   * @param demand The setpoint angle for Position PID control
   */
  protected void setSetpointPositionPID(Angle demand) {
    this.demand = demand.in(Degrees);
    if (mode != ControlMode.POSITION_PID) {
      mode = ControlMode.POSITION_PID;
    }
  }

  /** Stop the leader motor by setting open loop demand to 0 and stopping the motor */
  protected void stop() {
    setOpenLoop(0.0);
    leader.stopMotor();
  }

  /**
   * Get the current control mode of the leader motor
   *
   * @return The current control mode as a string, either "OPEN_LOOP", "MOTION_MAGIC", or
   *     "POSITION_PID"
   */
  protected String getControlMode() {
    return mode.toString();
  }

  protected double getError() {
    if (mode == ControlMode.MOTION_MAGIC || mode == ControlMode.POSITION_PID) {
      return demand - getPosition().in(Degrees);
    } else {
      return Double.NaN;
    }
  }

  /** Write the current demand to the leader motor based on the control mode */
  protected void writeToMotor() {
    if (mode == ControlMode.MOTION_MAGIC) {
      leader.setControl(
          new MotionMagicExpoVoltage(demand / 360)
              .withSlot(constants.motionMagicSlot)
              .withEnableFOC(false));
    } else if (mode == ControlMode.POSITION_PID) {
      leader.setControl(
          new PositionDutyCycle(demand / 360).withSlot(constants.positionSlot).withEnableFOC(true));
    } else {
      leader.setControl(new DutyCycleOut(demand));
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motorSignals);
    writeToMotor();
  }

  /**
   * Control modes for the servo subsystem
   *
   * <ul>
   *   <li>OPEN_LOOP: Open loop control
   *   <li>MOTION_MAGIC: Closed loop control using Motion Magic
   *   <li>POSITION_PID: Closed loop PID control
   * </ul>
   *
   * <p>Feedforward control (or “open-loop control”) refers to the class of algorithms which
   * incorporate knowledge of how the mechanism under control is expected to operate. Using this
   * “model” of operation, the control input is chosen to make the mechanism get close to where it
   * should be.
   *
   * <p>Feedback control (or “closed-loop control”) refers to the class of algorithms which use
   * sensors to measure what a mechanism is doing, and issue corrective commands to move a mechanism
   * from where it actually is, to where you want it to be.
   *
   * <p>https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/picking-control-strategy.html
   */
  protected enum ControlMode {
    OPEN_LOOP, // usually percent output
    MOTION_MAGIC,
    POSITION_PID
  }
}
