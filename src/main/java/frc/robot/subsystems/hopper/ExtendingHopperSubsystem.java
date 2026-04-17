package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.subsystems.ServoSubsystem;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import lombok.Getter;
import lombok.Setter;

public class ExtendingHopperSubsystem extends ServoSubsystem {

  @Getter @Setter private HopperState currentState = HopperState.RETRACTED;
  private HopperState lastState = currentState;

  private boolean homing = false;
  private boolean initializeHoming = false;
  private final DelayedBoolean velocityHomingDelay =
      new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
  private final DelayedBoolean currentHomingDelay =
      new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);

  private static ExtendingHopperSubsystem INSTANCE;

  public static ExtendingHopperSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ExtendingHopperSubsystem(HopperConstants.SERVO_CONSTANTS);
    }
    return INSTANCE;
  }

  private ExtendingHopperSubsystem(ServoSubsystemConstants constants) {
    super(constants);
    zeroMotor();
  }

  public Command setStateCommand(HopperState state) {
    return new InstantCommand(() -> currentState = state);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (currentState != lastState) {
      if (currentState.equals(HopperState.HOMING)) {
        homing = true;
        initializeHoming = true;
      } else if (currentState.equals(HopperState.FALLING)) {
        setOpenLoop(HopperConstants.OPEN_LOOP_FALLING_SPEED);
      } else {
        setSetpointMotionMagic(currentState.targetPosition);
      }

      lastState = currentState;
    }

    if (currentState.equals(HopperState.FALLING)
        && MathHelpers.epsilonEquals(
            leader.getPosition().getValue().in(Rotations),
            HopperConstants.MIN_HOPPER_POSITION.in(Rotations),
            0.5)) {
      setOpenLoop(0);
      currentState = HopperState.RETRACTED;
    }

    if (homing) {
      if (initializeHoming) {
        leader
            .getConfigurator()
            .apply(leaderConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(false));
        initializeHoming = false;
      }

      setOpenLoop(HopperConstants.OPEN_LOOP_HOMING_SPEED);

      // check to see if we've hit the bottom
      if (velocityHomingDelay.update(
              Timer.getFPGATimestamp(),
              MathHelpers.epsilonEquals(leader.getVelocity().getValueAsDouble(), 0.0, 0.5))
          && currentHomingDelay.update(
              Timer.getFPGATimestamp(), leader.getTorqueCurrent().getValue().abs(Amps) >= 10)) {

        // reset to bottom
        leader.setPosition(HopperConstants.MIN_HOPPER_POSITION);
        System.out.println("Hood Homed");
        homing = false;
        velocityHomingDelay.update(Timer.getFPGATimestamp(), false);
        currentHomingDelay.update(Timer.getFPGATimestamp(), false);
        leader
            .getConfigurator()
            .apply(leaderConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true));
      }
    }
  }

  public enum HopperState {
    EXTENDED(HopperConstants.EXTENDED_POSITION),
    RETRACTED(HopperConstants.RETRACTED_POSITION),
    FALLING(null),
    HOMING(null);

    private final Angle targetPosition;

    public Angle getTargetPosition() {
      return targetPosition;
    }

    private HopperState(Angle targetAngle) {
      this.targetPosition = targetAngle;
    }
  }
}
