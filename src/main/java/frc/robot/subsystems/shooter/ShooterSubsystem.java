package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.Setter;

public class ShooterSubsystem extends RollerSubsystem {
  private static ShooterSubsystem INSTANCE;

  @Getter @Setter private AngularVelocity goalPoint = RotationsPerSecond.of(0);
  private AngularVelocity lastGoal = RotationsPerSecond.of(0);
  @Getter @Setter private boolean runingOpenLoop = false;

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  }

  public ShooterSubsystem() {
    super(ShooterConstants.ROLLER_CONFIG);
  }

  @Override
  public Command runAtVelocityCommand(AngularVelocity velocity) {
    return Commands.runEnd(() -> setGoalPoint(velocity), () -> stopMotor(), this);
  }

  @Override
  public void stopMotor() {
    goalPoint = RotationsPerSecond.of(0);
    lastGoal = RotationsPerSecond.of(0);
    runingOpenLoop = false;
    super.stopMotor();
  }

  @Override
  public void periodic() {
    super.periodic();

    if (goalPoint.in(RotationsPerSecond) != lastGoal.in(RotationsPerSecond)) {
      runingOpenLoop = true;

      if (getVelocity().in(RotationsPerSecond) < goalPoint.in(RotationsPerSecond)) {
        setOpenLoop(ShooterConstants.BANG_BANG_SPEED);
      } else {
        setOpenLoop(-ShooterConstants.BANG_BANG_SPEED);
      }

      lastGoal = goalPoint;
    }

    // ATTENTION. IF WE ARE NOT RUNNING THE SHOOTER IT WILL RETURN HERE!!!!!!!!!!
    if (goalPoint.in(RotationsPerSecond) == 0) return;

    if ( // running open loop and within bounds
    runingOpenLoop
        && MathHelpers.epsilonEquals(
            getVelocity().in(RotationsPerSecond),
            goalPoint.in(RotationsPerSecond),
            ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {
      setGoalVelocity(goalPoint);
      runingOpenLoop = false;

    } else if ( // not running open loop but not within bounds
    !runingOpenLoop
        && !MathHelpers.epsilonEquals(
            getVelocity().in(RotationsPerSecond),
            goalPoint.in(RotationsPerSecond),
            ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {

      runingOpenLoop = true;
      if (getVelocity().in(RotationsPerSecond) < goalPoint.in(RotationsPerSecond)) {
        setOpenLoop(ShooterConstants.BANG_BANG_SPEED);
      } else {
        setOpenLoop(-ShooterConstants.BANG_BANG_SPEED);
      }
    } else if ( // running open loop and not in bounds
    runingOpenLoop
        && !MathHelpers.epsilonEquals(
            getVelocity().in(RotationsPerSecond),
            goalPoint.in(RotationsPerSecond),
            ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {

      if (leader.get() > 0
          && getVelocity().in(RotationsPerSecond) > goalPoint.in(RotationsPerSecond)) {
        // if running forwards but goal is behind us, reverse it
        setOpenLoop(-ShooterConstants.BANG_BANG_SPEED);
      } else if (leader.get() < 0
          && getVelocity().in(RotationsPerSecond) < goalPoint.in(RotationsPerSecond)) {
        // if running backwards but goal is in front of us, revese it
        setOpenLoop(ShooterConstants.BANG_BANG_SPEED);
      }
    }
  }
}
