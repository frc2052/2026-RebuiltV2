package com.team2052.lib.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  protected final TalonFX leader;
  protected final TalonFX[] followers;
  protected RollerSubsystemConstants constants;
  protected final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  // protected final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0).withSlot(0);
  protected final VoltageOut voltage = new VoltageOut(0);
  protected TalonFXConfiguration leaderConfig;
  protected TalonFXConfiguration[] followerConfigs;
  protected AngularVelocity goalVelocity;

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> torqueCurrentSignal;
  private final BaseStatusSignal[] signals;

  public RollerSubsystem(RollerSubsystemConstants constants) {
    super(constants.name);
    this.constants = constants;

    leader = new TalonFX(constants.leaderTalonFXConstants.id, constants.leaderTalonFXConstants.bus);

    leaderConfig = new TalonFXConfiguration();

    leaderConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechanismRatio;

    leaderConfig.Slot0.kP = constants.slot0kP;
    leaderConfig.Slot0.kI = constants.slot0kI;
    leaderConfig.Slot0.kD = constants.slot0kD;
    leaderConfig.Slot0.kV = constants.slot0kV;
    leaderConfig.Slot0.kA = constants.slot0kA;
    leaderConfig.Slot0.kS = constants.slot0kS;

    leaderConfig.MotorOutput.Inverted =
        (constants.counterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    leaderConfig.MotorOutput.NeutralMode = constants.neutralMode;

    leaderConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit.in(Amps);
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.enableSupplyCurrentLimit;
    leaderConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit.in(Amps);
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = constants.enableStatorCurrentLimit;

    leaderConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = constants.rampRate;
    leaderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = constants.rampRate;
    leaderConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = constants.rampRate;

    leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = constants.rampRate;
    leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = constants.rampRate;
    leaderConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = constants.rampRate;

    followers = new TalonFX[constants.followerTalonFXConstants.length];
    followerConfigs = new TalonFXConfiguration[constants.followerTalonFXConstants.length];

    for (int i = 0; i < followers.length; i++) {
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

      @SuppressWarnings("resource")
      TalonFX followerMotor = new TalonFX(fID, fBus);
      followerMotor.clearStickyFaults();
      followerMotor.setControl(
          new Follower(lID, fAligned).withUpdateFreqHz(constants.followerRateHz));

      followers[i] = followerMotor;
      TalonFXConfiguration followerConfig = new TalonFXConfiguration();
      followerMotor.getConfigurator().refresh(followerConfig);
      followerConfig.MotorOutput.NeutralMode = constants.neutralMode;
      followerConfig.MotorOutput.Inverted =
          (constants.counterClockwisePositive
              ? InvertedValue.CounterClockwise_Positive
              : InvertedValue.Clockwise_Positive);
      followerConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit.in(Amps);
      followerConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.enableSupplyCurrentLimit;
      followerConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit.in(Amps);
      followerConfig.CurrentLimits.StatorCurrentLimitEnable = constants.enableStatorCurrentLimit;
      followerMotor.getConfigurator().apply(followerConfig);
      followerConfigs[i] = followerConfig;
      followerMotor.optimizeBusUtilization();
    }

    leader.getConfigurator().apply(leaderConfig);

    leader.optimizeBusUtilization();

    velocitySignal = leader.getVelocity();
    torqueCurrentSignal = leader.getTorqueCurrent();
    signals = new BaseStatusSignal[] {velocitySignal, torqueCurrentSignal};
    BaseStatusSignal.setUpdateFrequencyForAll(200.0, signals);
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
   * Set the goal velocity of the roller subsystem. Will be clamped to the max velocity set in
   * constants if it is not 0.
   *
   * @param goalVelocity The desired velocity for the roller subsystem
   */
  public void setGoalVelocity(AngularVelocity goalVelocity) {
    this.goalVelocity = goalVelocity;
    if (constants.maxAngularVelocity.in(RotationsPerSecond) != 0) {
      this.goalVelocity =
          RotationsPerSecond.of(
              MathHelpers.clamp(
                  goalVelocity.in(RotationsPerSecond),
                  constants.maxAngularVelocity.unaryMinus().in(RotationsPerSecond),
                  constants.maxAngularVelocity.in(RotationsPerSecond)));
    }
    leader.setControl(velocityControl.withVelocity(this.goalVelocity));
    // Logger.recordOutput(
    // constants.name + "/Goal Velocity RPS", this.goalVelocity.in(RotationsPerSecond));
  }

  /**
   * Set the motor output as a percentage of the max velocity defined in constants. Will be clamped
   * to the max velocity if it is not 0.
   *
   * @param pct The desired motor output percentage, between -1 and 1
   */
  public void setMotorPct(double pct) {
    setGoalVelocity(constants.maxAngularVelocity.times(pct));
  }

  public void setOpenLoop(Voltage volts) {
    leader.setControl(voltage.withOutput(volts));
  }

  public void setOpenLoop(double output) {
    leader.setControl(new DutyCycleOut(output).withEnableFOC(true));
  }

  /** Stop the roller motors by setting the goal velocity to 0. */
  public void stopMotor() {
    setOpenLoop(Volts.of(0));
    goalVelocity = RotationsPerSecond.of(0);
  }

  /** Reverse the direction of the roller motors by negating the current goal velocity. */
  public void reverseMotor() {
    setGoalVelocity(goalVelocity.unaryMinus());
  }

  /**
   * Check if the roller subsystem is at the goal velocity within a specified margin.
   *
   * @param margin The allowable margin of error for the velocity
   * @return true if the current velocity is within the margin of the goal velocity, false otherwise
   */
  public boolean isAtGoalVelocity(AngularVelocity margin) {
    return Math.abs(getVelocity().in(RotationsPerSecond) - goalVelocity.in(RotationsPerSecond))
        < margin.in(RotationsPerSecond);
  }

  /**
   * Check if the roller subsystem is at the goal velocity within a specified percentage margin.
   *
   * @param pctMargin The allowable margin of error for the velocity as a percentage of max velocity
   * @return true if the current velocity is within the margin of the goal velocity, false otherwise
   */
  public boolean isAtGoalVelocity(double pctMargin) {
    return isAtGoalVelocity(constants.maxAngularVelocity.times(pctMargin));
  }

  /**
   * Check if the roller subsystem is at a specified velocity within a specified margin.
   *
   * @param goalVelocity The velocity to check against
   * @param margin The allowable margin of error for the velocity
   * @return true if the current velocity is within the margin of the specified velocity, false
   *     otherwise
   */
  public boolean isAtVelocity(AngularVelocity goalVelocity, AngularVelocity margin) {
    System.out.println(
        "shooter param v:"
            + getVelocity().in(RotationsPerSecond)
            + " g: "
            + goalVelocity.in(RotationsPerSecond));
    return Math.abs(getVelocity().in(RotationsPerSecond) - goalVelocity.in(RotationsPerSecond))
        < margin.in(RotationsPerSecond);
  }

  /**
   * Check if the roller subsystem is at a specified velocity within a specified percentage margin.
   *
   * @param pctGoalVelocity The velocity to check against as a percentage of max velocity
   * @param pctMargin The allowable margin of error for the velocity as a percentage of max velocity
   * @return true if the current velocity is within the margin of the specified velocity, false
   *     otherwise
   */
  public boolean isAtVelocity(double pctGoalVelocity, double pctMargin) {
    return isAtVelocity(
        constants.maxAngularVelocity.times(pctGoalVelocity),
        constants.maxAngularVelocity.times(pctMargin));
  }

  /**
   * Check if the roller subsystem is stopped (at 0 velocity) within a specified margin.
   *
   * @param margin The allowable margin of error for the velocity
   * @return true if the current velocity is within the margin of 0, false otherwise
   */
  public boolean isStopped(AngularVelocity margin) {
    return isAtVelocity(RotationsPerSecond.of(0), margin);
  }

  /**
   * Create a command that runs the roller at a specified velocity until the command is interrupted,
   * at which point it will stop the motor.
   *
   * @param velocity The desired velocity for the roller subsystem
   * @return A Command that runs the roller at the specified velocity
   */
  public Command runAtVelocityCommand(AngularVelocity velocity) {
    return Commands.runEnd(() -> setGoalVelocity(velocity), () -> stopMotor(), this);
  }

  /**
   * Create a command that runs the roller at a specified percentage of max velocity until the
   * command is interrupted, at which point it will stop the motor.
   *
   * @param pct The desired motor output percentage, between -1 and 1
   * @return A Command that runs the roller at the specified percentage of max velocity
   */
  public Command runAtPctCommand(double pct) {
    return Commands.runEnd(() -> setMotorPct(pct), () -> stopMotor(), this);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(signals);
  }
}
