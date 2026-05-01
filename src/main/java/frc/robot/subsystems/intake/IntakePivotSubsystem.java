package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemWithCANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakePivotSubsystem extends ServoSubsystemWithCANCoder {

  private Angle goalAngle = Rotations.of(0);
  private State setpointState = new State(goalAngle.in(Degrees), 0.0);

  private static IntakePivotSubsystem INSTANCE;

  public static IntakePivotSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE =
          new IntakePivotSubsystem(
              IntakeConstants.INTAKE_SERVO_CONSTANTS, IntakeConstants.INTAKE_ENCODER_CONSTANTS);
    }
    return INSTANCE;
  }

  private IntakePivotSubsystem(
      ServoSubsystemConstants constants, CANCoderConstants encoderConstants) {
    super(constants, encoderConstants);
  }

  public Command compressCommand() {
    return Commands.runEnd(
            () -> setOpenLoop(0.2),
            () ->
                Commands.sequence(
                    Commands.waitSeconds(0.5), setCommand(IntakePosition.OUT_POSITION)),
            this)
        .onlyWhile(() -> getPosition().lt(IntakePosition.STOW_POSITION.angle));
  }

  public Command setCommand(double degrees) {
    return Commands.runOnce(() -> set(Degrees.of(degrees)), this);
  }

  public Command setCommand(IntakePosition position) {
    return Commands.runOnce(() -> set(position), this);
  }

  public void set(IntakePosition position) {
    set(position.angle);
  }

  public void set(Angle target) {
    goalAngle = boundAngle(target);
    setSetpointMotionMagic(goalAngle);
  }

  public Angle boundAngle(Angle angle) {
    return Degrees.of(
        MathUtil.clamp(
            angle.in(Degrees),
            IntakeConstants.MIN_INTAKE_ARM_ANGLE.in(Degrees),
            IntakeConstants.MAX_INTAKE_ARM_ANGLE.in(Degrees)));
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    //   goalAngle = getPosition();
    // }

    // double previousGoal = setpointState.position;

    // if (previousGoal != goalAngle.in(Degrees) || !getPosition().isNear(goalAngle,
    // IntakeConstants.POSITION_TOLERANCE_ANGLE)) {
    // State goalState = new State(goalAngle.in(Degrees), 0);
    // setpointState = goalState;
    // setSetpointMotionMagic(Degrees.of(setpointState.position));
    // }

    super.periodic();
  }

  public enum IntakePosition {
    IN_POSITION(IntakeConstants.UP_POSITION),
    DEPOT_POSITION(IntakeConstants.DEPOT_POSITION),
    STOW_POSITION(IntakeConstants.STOW_POSITION),
    HALFWAY_POSITION(IntakeConstants.HALFWAY_POSITION),
    OUT_POSITION(IntakeConstants.DOWN_POSITION);

    Angle angle;

    private IntakePosition(Angle angle) {
      this.angle = angle;
    }
  }
}
