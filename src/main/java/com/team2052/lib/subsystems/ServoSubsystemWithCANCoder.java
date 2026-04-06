package com.team2052.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ServoSubsystemWithCANCoder extends ServoSubsystem {
  private final CANcoder encoder;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final BaseStatusSignal[] encoderSignals;

  public ServoSubsystemWithCANCoder(
      ServoSubsystemConstants constants, CANCoderConstants encoderConstants) {
    super(constants);

    encoder = new CANcoder(encoderConstants.id.getFirst(), encoderConstants.id.getSecond());
    encoder.getConfigurator().apply(encoderConstants.config);
    positionSignal = encoder.getAbsolutePosition();
    velocitySignal = encoder.getVelocity();

    encoderSignals = new BaseStatusSignal[] {positionSignal, velocitySignal};

    encoder.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        encoderConstants.statusSignalUpdateFrequency, encoderSignals);

    leaderConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    leaderConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechanismRatio;
    leaderConfig.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio;
    leader.getConfigurator().apply(leaderConfig);
    encoder.setPosition(encoder.getAbsolutePosition().getValue());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(encoderSignals);
    super.periodic();
  }

  public void resetEncoderTo(Angle newAngle) {
    encoder.setPosition(newAngle);
  }
}
