package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemWithCANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
    return Commands.runOnce(() -> setOpenLoop(0.1), this)
        .until(
            () ->
                getPosition()
                    .isNear(
                        IntakePosition.STOW_POSITION.angle,
                        IntakeConstants.POSITION_TOLERANCE_ANGLE));
  }

  public Command setCommand(double degrees) {
    return new InstantCommand(() -> set(Degrees.of(degrees)));
  }

  public Command setCommand(IntakePosition position) {
    return new InstantCommand(() -> set(position));
  }

  public void set(IntakePosition position) {
    set(position.angle);
  }

  public void set(Angle target) {
    goalAngle = boundAngle(target);
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
    if (DriverStation.isDisabled()) {
      goalAngle = getPosition();
    }

    double previousGoal = setpointState.position;

    if (previousGoal != goalAngle.in(Degrees)) {
      State goalState = new State(goalAngle.in(Degrees), 0);
      setpointState = goalState;
      setSetpointMotionMagic(Degrees.of(setpointState.position));
    }

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
