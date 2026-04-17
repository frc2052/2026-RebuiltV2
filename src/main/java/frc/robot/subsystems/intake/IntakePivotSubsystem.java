package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemWithCANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
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

  public enum IntakePosition {
    IN_POSITION(IntakeConstants.IN_POSITION),
    STOW_POSITION(IntakeConstants.STOW_POSITION),
    HALFWAY_POSITION(IntakeConstants.HALFWAY_POSITION),
    OUT_POSITION(IntakeConstants.OUT_POSITION);

    Angle angle;

    private IntakePosition(Angle angle) {
      this.angle = angle;
    }
  }
}
